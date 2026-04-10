#define _DEFAULT_SOURCE

#include "dev.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "log.h"
#include "proto_zdt_arm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM_JOINT_COUNT 6U
#define ARM_DEFAULT_RPM 300U
#define ARM_DEFAULT_ACC 20U
/* home_mode byte in 0x9A trigger-home command (ZDT step motor protocol):
 *   0 = single-side limit-switch homing
 *   1 = double-side limit-switch homing
 *   2 = collision / stall-detection homing
 *   3 = closed-loop position homing
 * Must MATCH ARM_ZERO_MODE_DEFAULT below; otherwise the motor rejects 0x9A
 * with status 0xE2 because the requested mode contradicts what 0x4C set up. */
#define ARM_HOME_MODE_DEFAULT 2U
#define ARM_MOTOR_PULSES_PER_REV 3200.0
#define ARM_CAN_DEFAULT_IFACE "can4"
#define ARM_DEFAULT_ACK_TIMEOUT_MS 200
#define ARM_DEFAULT_REACHED_TIMEOUT_MS 8000
/* zero_mode byte in 0x4C write-zero-params command:
 *   0 = disabled  1 = end-stop switch  2 = stall / collision detection
 * Set to 2 so the motor drives until it physically stalls, not to position 0. */
#define ARM_ZERO_MODE_DEFAULT 2U
/* zero_direction byte in 0x4C write-zero-params command:
 *   0 = forward (CW)  - same direction as positive move targets
 *   1 = reverse (CCW) - opposite direction to positive move targets
 * Default reverse so homing drives the joint AWAY from typical workspace
 * positions and into a hard stop. Override per-joint via UAV_ARM_J<n>_ZERO_DIR
 * or globally via UAV_ARM_ZERO_DIRECTION. */
#define ARM_ZERO_DIRECTION_DEFAULT 1U
#define ARM_ZERO_SPEED_DEFAULT_RPM 30U
#define ARM_ZERO_TIMEOUT_DEFAULT_MS 10000U
#define ARM_COLLISION_ZERO_SPEED_DEFAULT_RPM 300U
#define ARM_COLLISION_ZERO_CURRENT_DEFAULT_MA 800U
#define ARM_COLLISION_ZERO_TIME_DEFAULT_MS 60U
/* enable_auto_zero: 1 = motor automatically sets zero point after stall detection.
 * Required so the stall position becomes the new absolute origin for later moves. */
#define ARM_AUTO_ZERO_DEFAULT 1U

#define ARM_BASE_OFFSET_MM 55.0
#define ARM_BASE_HEIGHT_MM 166.0
#define ARM_LINK2_MM 200.0
#define ARM_LINK3_A_MM 56.0
#define ARM_LINK4_D_MM 192.0
#define ARM_TOOL_OFFSET_MM 55.0

typedef struct {
    uint8_t addr;
    double ratio;
    double min_deg;
    double max_deg;
    int dir_sign;
    int zero_offset_pulses;
} ArmJointConfig;

typedef struct {
    ArmJointConfig cfg;
    double target_deg;
} ArmJointTarget;

typedef struct {
    double target_deg[ARM_JOINT_COUNT];
} ArmPostHomeTargets;

static int g_arm_can_fd = -1;
static char g_arm_can_iface[IF_NAMESIZE] = ARM_CAN_DEFAULT_IFACE;

/* Runtime speed overrides set via arm_set_move_rpm / arm_set_zero_rpm.
 * A value of 0 means "no override - fall back to env var / compiled default".
 * The HostGUI ArmWidget pushes new values into these via the
 * arm.set_speeds RPC whenever the user edits the speed fields. */
static int g_arm_move_rpm_override = 0;
static int g_arm_zero_rpm_override = 0;

/* Emergency-stop signaling. The atomic flag can be set from any thread (e.g.
 * proc_arm's watchdog) to interrupt blocking arm_wait_response calls. The
 * self-pipe lets poll() in arm_receive_response_once wake up immediately on
 * estop, instead of waiting for its full timeout.
 *
 * We use GCC __atomic_* builtins (instead of <stdatomic.h>) so this file
 * compiles cleanly under both C (uav_robotd) and C++ (proc_arm). */
static volatile int g_arm_estop_flag = 0;
static int g_arm_estop_pipe[2] = {-1, -1};

static int arm_estop_init_pipe(void) {
    int flags;
    if (g_arm_estop_pipe[0] >= 0) {
        return 0;
    }
    if (pipe(g_arm_estop_pipe) < 0) {
        log_error("core.dev.arm", "estop pipe create failed: %s", strerror(errno));
        g_arm_estop_pipe[0] = g_arm_estop_pipe[1] = -1;
        return -1;
    }
    flags = fcntl(g_arm_estop_pipe[0], F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(g_arm_estop_pipe[0], F_SETFL, flags | O_NONBLOCK);
    }
    flags = fcntl(g_arm_estop_pipe[1], F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(g_arm_estop_pipe[1], F_SETFL, flags | O_NONBLOCK);
    }
    return 0;
}

void arm_request_estop(void) {
    __atomic_store_n(&g_arm_estop_flag, 1, __ATOMIC_RELEASE);
    if (g_arm_estop_pipe[1] < 0) {
        (void)arm_estop_init_pipe();
    }
    if (g_arm_estop_pipe[1] >= 0) {
        char b = 'S';
        ssize_t r = write(g_arm_estop_pipe[1], &b, 1);
        (void)r;
    }
    log_warn("core.dev.arm", "arm_request_estop: estop flag set");
}

void arm_clear_estop(void) {
    __atomic_store_n(&g_arm_estop_flag, 0, __ATOMIC_RELEASE);
    if (g_arm_estop_pipe[0] >= 0) {
        char buf[64];
        while (read(g_arm_estop_pipe[0], buf, sizeof(buf)) > 0) {
        }
    }
}

bool arm_estop_requested(void) {
    return __atomic_load_n(&g_arm_estop_flag, __ATOMIC_ACQUIRE) != 0;
}

static int arm_round_to_int(double value) {
    if (value >= 0.0) {
        return (int)(value + 0.5);
    }
    return (int)(value - 0.5);
}

static int arm_getenv_int(const char *name, int fallback) {
    const char *text = getenv(name);
    char *end = NULL;
    long value;

    if (text == NULL || text[0] == '\0') {
        return fallback;
    }

    value = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return fallback;
    }
    return (int)value;
}

static double arm_getenv_double(const char *name, double fallback) {
    const char *text = getenv(name);
    char *end = NULL;
    double value;

    if (text == NULL || text[0] == '\0') {
        return fallback;
    }

    value = strtod(text, &end);
    if (end == text || *end != '\0') {
        return fallback;
    }
    return value;
}

static double arm_rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

static double arm_deg_clamp(double value, double min_deg, double max_deg) {
    if (value < min_deg) return min_deg;
    if (value > max_deg) return max_deg;
    return value;
}

static uint16_t arm_get_command_rpm(void) {
    int rpm;
    if (g_arm_move_rpm_override > 0) {
        rpm = g_arm_move_rpm_override;
    } else {
        rpm = arm_getenv_int("UAV_ARM_RPM", ARM_DEFAULT_RPM);
    }
    if (rpm < 1) {
        rpm = 1;
    }
    if (rpm > 3000) {
        rpm = 3000;
    }
    return (uint16_t)rpm;
}

void arm_set_move_rpm(int rpm) {
    g_arm_move_rpm_override = (rpm > 0) ? rpm : 0;
    log_info("core.dev.arm", "arm_set_move_rpm -> %d", g_arm_move_rpm_override);
}

void arm_set_zero_rpm(int rpm) {
    g_arm_zero_rpm_override = (rpm > 0) ? rpm : 0;
    log_info("core.dev.arm", "arm_set_zero_rpm -> %d", g_arm_zero_rpm_override);
}

int arm_get_move_rpm(void) {
    if (g_arm_move_rpm_override > 0) {
        return g_arm_move_rpm_override;
    }
    return arm_getenv_int("UAV_ARM_RPM", ARM_DEFAULT_RPM);
}

int arm_get_zero_rpm(void) {
    if (g_arm_zero_rpm_override > 0) {
        return g_arm_zero_rpm_override;
    }
    return arm_getenv_int("UAV_ARM_ZERO_SPEED_RPM", ARM_ZERO_SPEED_DEFAULT_RPM);
}

static uint8_t arm_get_command_acc(void) {
    int acc = arm_getenv_int("UAV_ARM_ACC", ARM_DEFAULT_ACC);
    if (acc < 0) {
        acc = 0;
    }
    if (acc > 255) {
        acc = 255;
    }
    return (uint8_t)acc;
}

static uint8_t arm_get_home_mode(void) {
    int mode = arm_getenv_int("UAV_ARM_HOME_MODE", ARM_HOME_MODE_DEFAULT);
    if (mode < 0) {
        mode = 0;
    }
    if (mode > 255) {
        mode = 255;
    }
    return (uint8_t)mode;
}

static int arm_get_home_settle_ms(void) {
    int ms = arm_getenv_int("UAV_ARM_HOME_SETTLE_MS", 2500);
    if (ms < 0) {
        ms = 0;
    }
    if (ms > 60000) {
        ms = 60000;
    }
    return ms;
}

static int arm_get_post_home_move_delay_ms(void) {
    int ms = arm_getenv_int("UAV_ARM_POST_HOME_MOVE_DELAY_MS", 500);
    if (ms < 0) {
        ms = 0;
    }
    if (ms > 60000) {
        ms = 60000;
    }
    return ms;
}

/* Look up the zero (homing) direction for a specific joint.
 *
 * Resolution order (first one set wins):
 *   1. UAV_ARM_J<n>_ZERO_DIR  - per-joint override (n = 1..6)
 *   2. UAV_ARM_ZERO_DIRECTION - global override
 *   3. ARM_ZERO_DIRECTION_DEFAULT - compiled-in default
 *
 * Accepted values: 0 = forward (CW), 1 = reverse (CCW). */
static uint8_t arm_get_zero_direction_for_joint(uint8_t addr) {
    if (addr >= 1U && addr <= ARM_JOINT_COUNT) {
        char env_name[32];
        snprintf(env_name, sizeof(env_name), "UAV_ARM_J%u_ZERO_DIR", (unsigned)addr);
        const char *per_joint = getenv(env_name);
        if (per_joint != NULL && per_joint[0] != '\0') {
            return (uint8_t)arm_getenv_int(env_name, ARM_ZERO_DIRECTION_DEFAULT);
        }
    }
    return (uint8_t)arm_getenv_int("UAV_ARM_ZERO_DIRECTION",
                                   ARM_ZERO_DIRECTION_DEFAULT);
}

static ZdtArmZeroParams arm_get_zero_params_for_joint(uint8_t addr) {
    ZdtArmZeroParams params;

    params.zero_mode = (uint8_t)arm_getenv_int("UAV_ARM_ZERO_MODE", ARM_ZERO_MODE_DEFAULT);
    params.zero_direction = arm_get_zero_direction_for_joint(addr);
    /* Honor a runtime override (set via arm_set_zero_rpm) before falling
     * back to the env var or compiled default. */
    if (g_arm_zero_rpm_override > 0) {
        params.zero_speed_rpm = (uint16_t)g_arm_zero_rpm_override;
    } else {
        params.zero_speed_rpm = (uint16_t)arm_getenv_int("UAV_ARM_ZERO_SPEED_RPM", ARM_ZERO_SPEED_DEFAULT_RPM);
    }
    params.zero_timeout_ms = (uint32_t)arm_getenv_int("UAV_ARM_ZERO_TIMEOUT_MS", ARM_ZERO_TIMEOUT_DEFAULT_MS);
    params.collision_zero_speed_rpm = (uint16_t)arm_getenv_int("UAV_ARM_COLLISION_ZERO_SPEED_RPM", ARM_COLLISION_ZERO_SPEED_DEFAULT_RPM);
    params.collision_zero_current_ma = (uint16_t)arm_getenv_int("UAV_ARM_COLLISION_ZERO_CURRENT_MA", ARM_COLLISION_ZERO_CURRENT_DEFAULT_MA);
    params.collision_zero_time_ms = (uint16_t)arm_getenv_int("UAV_ARM_COLLISION_ZERO_TIME_MS", ARM_COLLISION_ZERO_TIME_DEFAULT_MS);
    params.enable_auto_zero = arm_getenv_int("UAV_ARM_ENABLE_AUTO_ZERO", ARM_AUTO_ZERO_DEFAULT) != 0;
    return params;
}


static ArmPostHomeTargets arm_get_post_home_targets(void) {
    static const double k_default_targets[ARM_JOINT_COUNT] = {180.0, 90.0, 83.0, 30.0, 110.0, 30.0};
    static const char *k_target_env[ARM_JOINT_COUNT] = {
        "UAV_ARM_J1_POST_HOME_DEG", "UAV_ARM_J2_POST_HOME_DEG", "UAV_ARM_J3_POST_HOME_DEG",
        "UAV_ARM_J4_POST_HOME_DEG", "UAV_ARM_J5_POST_HOME_DEG", "UAV_ARM_J6_POST_HOME_DEG"
    };
    ArmPostHomeTargets targets;
    size_t i;

    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        targets.target_deg[i] = arm_getenv_double(k_target_env[i], k_default_targets[i]);
    }
    return targets;
}

static int arm_can_init(void) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    const char *iface = getenv("UAV_ARM_CAN_IFACE");
    int fd;

    if (g_arm_can_fd >= 0) {
        return 0;
    }

    if (iface != NULL && iface[0] != '\0') {
        (void)snprintf(g_arm_can_iface, sizeof(g_arm_can_iface), "%s", iface);
    }

    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        log_error("core.dev.arm", "socket(PF_CAN) failed: %s", strerror(errno));
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    (void)snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", g_arm_can_iface);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        log_error("core.dev.arm",
                  "SIOCGIFINDEX %s failed: %s",
                  g_arm_can_iface,
                  strerror(errno));
        close(fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("core.dev.arm",
                  "bind(%s) failed: %s",
                  g_arm_can_iface,
                  strerror(errno));
        close(fd);
        return -1;
    }

    g_arm_can_fd = fd;
    log_info("core.dev.arm", "socketcan ready on %s", g_arm_can_iface);
    return 0;
}

static bool arm_send_can_frame(const ZdtArmCanFrame *frame) {
    struct can_frame can_frame_data;
    ssize_t written;

    if (frame == NULL) {
        return false;
    }
    if (arm_can_init() != 0) {
        return false;
    }

    memset(&can_frame_data, 0, sizeof(can_frame_data));
    if (frame->is_extended_id) {
        can_frame_data.can_id = (canid_t)(frame->can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    } else {
        can_frame_data.can_id = (canid_t)(frame->can_id & CAN_SFF_MASK);
    }
    can_frame_data.can_dlc = frame->len;
    memcpy(can_frame_data.data, frame->data, frame->len);

    written = write(g_arm_can_fd, &can_frame_data, sizeof(can_frame_data));
    if (written != (ssize_t)sizeof(can_frame_data)) {
        log_error("core.dev.arm",
                  "write(can_id=0x%08X) failed: %s",
                  (unsigned int)frame->can_id,
                  strerror(errno));
        return false;
    }
    return true;
}

static void arm_drain_rx(void) {
    if (arm_can_init() != 0) {
        return;
    }

    while (1) {
        struct can_frame frame;
        ssize_t n = recv(g_arm_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            return;
        }
        if (n == 0) {
            return;
        }
    }
}

static bool arm_receive_response_once(uint8_t *out_addr,
                                      ZdtArmResponse *out_resp,
                                      int timeout_ms) {
    struct pollfd pfd[2];
    struct can_frame frame;
    ssize_t nread;
    ZdtArmResponse resp;
    uint8_t addr = 0U;
    int nfds = 1;
    int prc;

    if (out_addr == NULL || out_resp == NULL) {
        return false;
    }
    if (arm_can_init() != 0) {
        return false;
    }

    /* Fast-path: if estop already requested, do not enter another wait. */
    if (__atomic_load_n(&g_arm_estop_flag, __ATOMIC_ACQUIRE)) {
        return false;
    }

    (void)arm_estop_init_pipe();

    memset(pfd, 0, sizeof(pfd));
    pfd[0].fd = g_arm_can_fd;
    pfd[0].events = POLLIN;
    if (g_arm_estop_pipe[0] >= 0) {
        pfd[1].fd = g_arm_estop_pipe[0];
        pfd[1].events = POLLIN;
        nfds = 2;
    }

    prc = poll(pfd, (nfds_t)nfds, timeout_ms);
    if (prc <= 0) {
        return false;
    }
    if (nfds == 2 && (pfd[1].revents & POLLIN) != 0) {
        /* Estop pipe signaled - abort immediately. The flag check on the
         * next call will keep returning false until arm_clear_estop. */
        log_warn("core.dev.arm", "arm wait aborted by estop signal");
        return false;
    }
    if ((pfd[0].revents & POLLIN) == 0) {
        return false;
    }

    nread = recv(g_arm_can_fd, &frame, sizeof(frame), 0);
    if (nread != (ssize_t)sizeof(frame)) {
        return false;
    }
    if ((frame.can_id & CAN_EFF_FLAG) != 0) {
        addr = (uint8_t)(((uint32_t)(frame.can_id & CAN_EFF_MASK) >> 8) & 0xFFU);
    }
    if (!proto_zdt_arm_decode(frame.data, frame.can_dlc, &resp)) {
        return false;
    }
    if (resp.addr == 0U) {
        resp.addr = addr;
    }

    *out_addr = resp.addr;
    *out_resp = resp;
    return true;
}

static bool arm_wait_response(uint8_t addr,
                              uint8_t expected_cmd,
                              ZdtArmResponseType expected_type,
                              int timeout_ms) {
    ZdtArmResponse resp;
    uint8_t resp_addr = 0U;

    while (arm_receive_response_once(&resp_addr, &resp, timeout_ms)) {
        if (addr != 0U && resp_addr != 0U && resp_addr != addr) {
            continue;
        }
        if (resp.cmd != expected_cmd) {
            continue;
        }
        if (resp.type == ZDT_ARM_RESP_ACK_REJECTED || resp.type == ZDT_ARM_RESP_ACK_INVALID) {
            log_error("core.dev.arm",
                      "arm cmd=0x%02X addr=%u rejected status=0x%02X",
                      (unsigned int)resp.cmd,
                      (unsigned int)addr,
                      (unsigned int)resp.status);
            return false;
        }
        if (resp.type == expected_type ||
            (expected_type == ZDT_ARM_RESP_ACK_OK && resp.type == ZDT_ARM_RESP_UNKNOWN)) {
            return true;
        }
    }

    log_warn("core.dev.arm",
             "arm wait response timeout addr=%u cmd=0x%02X type=%d",
             (unsigned int)addr,
             (unsigned int)expected_cmd,
             (int)expected_type);
    return false;
}

static bool arm_wait_ack(uint8_t addr, uint8_t expected_cmd) {
    return arm_wait_response(addr,
                             expected_cmd,
                             ZDT_ARM_RESP_ACK_OK,
                             arm_getenv_int("UAV_ARM_ACK_TIMEOUT_MS", ARM_DEFAULT_ACK_TIMEOUT_MS));
}

static bool arm_wait_sync_ack(uint8_t addr) {
    return arm_wait_ack(addr, 0xFFU);
}

static bool arm_wait_position_reached(uint8_t addr) {
    return arm_wait_response(addr,
                             0xFDU,
                             ZDT_ARM_RESP_POSITION_REACHED,
                             arm_getenv_int("UAV_ARM_REACHED_TIMEOUT_MS", ARM_DEFAULT_REACHED_TIMEOUT_MS));
}

static bool arm_wait_multi_joint_reached(void) {
    size_t reached_count = 0U;
    bool reached_mask[ARM_JOINT_COUNT];

    memset(reached_mask, 0, sizeof(reached_mask));
    while (reached_count < ARM_JOINT_COUNT) {
        ZdtArmResponse resp;
        uint8_t addr = 0U;
        size_t idx;

        if (!arm_receive_response_once(&addr,
                                       &resp,
                                       arm_getenv_int("UAV_ARM_REACHED_TIMEOUT_MS", ARM_DEFAULT_REACHED_TIMEOUT_MS))) {
            log_warn("core.dev.arm", "multi-joint wait reached timeout");
            return false;
        }
        if (resp.cmd != 0xFDU || resp.type != ZDT_ARM_RESP_POSITION_REACHED) {
            if (resp.type == ZDT_ARM_RESP_ACK_REJECTED || resp.type == ZDT_ARM_RESP_ACK_INVALID) {
                return false;
            }
            continue;
        }
        if (addr < 1U || addr > ARM_JOINT_COUNT) {
            continue;
        }
        idx = (size_t)(addr - 1U);
        if (!reached_mask[idx]) {
            reached_mask[idx] = true;
            reached_count++;
        }
    }
    return true;
}

static bool arm_should_wait_reached(void) {
    return arm_getenv_int("UAV_ARM_WAIT_REACHED", 0) != 0;
}

static bool arm_require_enable_ack(void) {
    return arm_getenv_int("UAV_ARM_REQUIRE_ENABLE_ACK", 0) != 0;
}

static bool arm_require_zero_params_ack(void) {
    return arm_getenv_int("UAV_ARM_REQUIRE_ZERO_PARAMS_ACK", 0) != 0;
}

static bool arm_send_batch(const ZdtArmCanBatch *batch) {
    size_t i;

    if (batch == NULL) {
        return false;
    }
    for (i = 0; i < batch->count; ++i) {
        if (!arm_send_can_frame(&batch->frames[i])) {
            return false;
        }
    }
    return true;
}

static bool arm_enable_joint(uint8_t addr) {
    ZdtArmCanBatch batch;

    if (!proto_zdt_arm_encode_enable(addr, true, false, &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    return arm_wait_ack(addr, 0xF3U);
}

static bool arm_write_zero_params(uint8_t addr) {
    const ZdtArmZeroParams params = arm_get_zero_params_for_joint(addr);
    const bool save = arm_getenv_int("UAV_ARM_ZERO_SAVE", 1) != 0;
    ZdtArmCanBatch batch;

    if (!proto_zdt_arm_encode_write_zero_params(addr, save, &params, &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (!arm_wait_ack(addr, 0x4CU)) {
        if (arm_require_zero_params_ack()) {
            log_warn("core.dev.arm",
                     "arm zero params addr=%u ack missed, abort by policy",
                     (unsigned int)addr);
            return false;
        }
        log_warn("core.dev.arm",
                 "arm zero params addr=%u ack missed, continue with existing/latched params",
                 (unsigned int)addr);
        return true;
    }

    log_info("core.dev.arm",
             "arm zero params addr=%u mode=%u dir=%u zero_speed=%u timeout_ms=%u collision_speed=%u collision_current=%u collision_time=%u save=%d",
             (unsigned int)addr,
             (unsigned int)params.zero_mode,
             (unsigned int)params.zero_direction,
             (unsigned int)params.zero_speed_rpm,
             (unsigned int)params.zero_timeout_ms,
             (unsigned int)params.collision_zero_speed_rpm,
             (unsigned int)params.collision_zero_current_ma,
             (unsigned int)params.collision_zero_time_ms,
             save ? 1 : 0);
    return true;
}

static ArmJointConfig arm_joint_config(size_t joint_index) {
    static const double k_default_ratios[ARM_JOINT_COUNT] = {25.0, 20.0, 25.0, 10.0, 4.0, 1.0};
    static const double k_default_min_deg[ARM_JOINT_COUNT] = {-180.0, 0.0, -80.0, -30.0, -110.0, -30.0};
    static const double k_default_max_deg[ARM_JOINT_COUNT] = {180.0, 180.0, 83.0, 305.0, 110.0, 305.0};
    static const int k_default_dirs[ARM_JOINT_COUNT] = {1, 1, 1, 1, 1, 1};
    static const char *k_ratio_env[ARM_JOINT_COUNT] = {
        "UAV_ARM_J1_RATIO", "UAV_ARM_J2_RATIO", "UAV_ARM_J3_RATIO",
        "UAV_ARM_J4_RATIO", "UAV_ARM_J5_RATIO", "UAV_ARM_J6_RATIO"
    };
    static const char *k_dir_env[ARM_JOINT_COUNT] = {
        "UAV_ARM_J1_DIR", "UAV_ARM_J2_DIR", "UAV_ARM_J3_DIR",
        "UAV_ARM_J4_DIR", "UAV_ARM_J5_DIR", "UAV_ARM_J6_DIR"
    };
    static const char *k_zero_env[ARM_JOINT_COUNT] = {
        "UAV_ARM_J1_ZERO_OFFSET", "UAV_ARM_J2_ZERO_OFFSET", "UAV_ARM_J3_ZERO_OFFSET",
        "UAV_ARM_J4_ZERO_OFFSET", "UAV_ARM_J5_ZERO_OFFSET", "UAV_ARM_J6_ZERO_OFFSET"
    };
    static const char *k_min_env[ARM_JOINT_COUNT] = {
        "UAV_ARM_J1_MIN_DEG", "UAV_ARM_J2_MIN_DEG", "UAV_ARM_J3_MIN_DEG",
        "UAV_ARM_J4_MIN_DEG", "UAV_ARM_J5_MIN_DEG", "UAV_ARM_J6_MIN_DEG"
    };
    static const char *k_max_env[ARM_JOINT_COUNT] = {
        "UAV_ARM_J1_MAX_DEG", "UAV_ARM_J2_MAX_DEG", "UAV_ARM_J3_MAX_DEG",
        "UAV_ARM_J4_MAX_DEG", "UAV_ARM_J5_MAX_DEG", "UAV_ARM_J6_MAX_DEG"
    };
    ArmJointConfig cfg;

    cfg.addr = (uint8_t)(joint_index + 1U);
    cfg.ratio = arm_getenv_double(k_ratio_env[joint_index], k_default_ratios[joint_index]);
    cfg.min_deg = arm_getenv_double(k_min_env[joint_index], k_default_min_deg[joint_index]);
    cfg.max_deg = arm_getenv_double(k_max_env[joint_index], k_default_max_deg[joint_index]);
    cfg.dir_sign = arm_getenv_int(k_dir_env[joint_index], k_default_dirs[joint_index]);
    cfg.zero_offset_pulses = arm_getenv_int(k_zero_env[joint_index], 0);
    if (cfg.dir_sign >= 0) {
        cfg.dir_sign = 1;
    } else {
        cfg.dir_sign = -1;
    }
    if (cfg.max_deg < cfg.min_deg) {
        const double tmp = cfg.max_deg;
        cfg.max_deg = cfg.min_deg;
        cfg.min_deg = tmp;
    }
    return cfg;
}

static bool arm_plan_joint_position(uint8_t addr,
                                    double ratio,
                                    int dir_sign,
                                    int zero_offset_pulses,
                                    double target_deg,
                                    bool sync,
                                    ZdtArmCanBatch *out_batch) {
    double pulses_per_degree = (ARM_MOTOR_PULSES_PER_REV * ratio) / 360.0;
    int signed_pulses;
    bool ccw;

    if (out_batch == NULL) {
        return false;
    }

    if (!arm_enable_joint(addr)) {
        if (arm_require_enable_ack()) {
            log_warn("core.dev.arm",
                     "arm move joint %u enable ack missed, abort by policy",
                     (unsigned int)addr);
            return false;
        }
        log_warn("core.dev.arm",
                 "arm move joint %u enable ack missed, continue with position command",
                 (unsigned int)addr);
    }

    signed_pulses = arm_round_to_int(target_deg * pulses_per_degree * (double)dir_sign) +
                    zero_offset_pulses;
    ccw = signed_pulses < 0;
    if (signed_pulses < 0) {
        signed_pulses = -signed_pulses;
    }

    return proto_zdt_arm_encode_position(addr,
                                         ccw,
                                         arm_get_command_rpm(),
                                         arm_get_command_acc(),
                                         (uint32_t)signed_pulses,
                                         true,
                                         sync,
                                         out_batch);
}

static bool arm_plan_joint_absolute(ArmJointConfig cfg,
                                    double target_deg,
                                    bool sync,
                                    ZdtArmCanBatch *out_batch) {
    if (out_batch == NULL) {
        return false;
    }
    if (target_deg < cfg.min_deg || target_deg > cfg.max_deg) {
        log_warn("core.dev.arm",
                 "joint addr=%u target %.2fdeg out of range [%.2f, %.2f]",
                 (unsigned int)cfg.addr,
                 target_deg,
                 cfg.min_deg,
                 cfg.max_deg);
        return false;
    }
    return arm_plan_joint_position(cfg.addr,
                                   cfg.ratio,
                                   cfg.dir_sign,
                                   cfg.zero_offset_pulses,
                                   target_deg,
                                   sync,
                                   out_batch);
}

static bool arm_move_single_joint_absolute(ArmJointConfig cfg, double target_deg) {
    ZdtArmCanBatch batch;

    if (!arm_plan_joint_absolute(cfg, target_deg, false, &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (arm_should_wait_reached() && !arm_wait_position_reached(cfg.addr)) {
        return false;
    }
    return true;
}

static bool arm_move_joints_deg_impl(const double joints_deg[ARM_JOINT_COUNT]) {
    ArmJointTarget targets[ARM_JOINT_COUNT];
    ZdtArmCanBatch batch;
    size_t i;

    if (joints_deg == NULL) {
        return false;
    }

    arm_drain_rx();
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        targets[i].cfg = arm_joint_config(i);
        targets[i].target_deg = joints_deg[i];
    }

    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (!arm_plan_joint_absolute(targets[i].cfg,
                                     targets[i].target_deg,
                                     true,
                                     &batch)) {
            return false;
        }
        if (!arm_send_batch(&batch)) {
            return false;
        }
    }

    if (!proto_zdt_arm_encode_sync_start(&batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (!arm_wait_sync_ack(0U)) {
        return false;
    }
    if (arm_should_wait_reached()) {
        return arm_wait_multi_joint_reached();
    }
    return true;
}

static bool arm_home_all_joints(void) {
    ZdtArmCanBatch batch;
    const ArmPostHomeTargets post_home_targets = arm_get_post_home_targets();
    size_t i;
    const uint8_t home_mode = arm_get_home_mode();
    const int settle_ms = arm_get_home_settle_ms();
    const int post_home_move_delay_ms = arm_get_post_home_move_delay_ms();

    arm_drain_rx();
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        const uint8_t addr = (uint8_t)(i + 1U);
        const ArmJointConfig cfg = arm_joint_config(i);
        const double target_deg = post_home_targets.target_deg[i];
        if (!arm_enable_joint(addr)) {
            log_warn("core.dev.arm",
                     "arm home joint %u enable ack missed, continue with home trigger",
                     (unsigned int)addr);
        }
        if (!arm_write_zero_params(addr)) {
            log_warn("core.dev.arm",
                     "arm home joint %u zero params write failed",
                     (unsigned int)addr);
            return false;
        }
        if (!proto_zdt_arm_encode_trigger_home(addr,
                                               home_mode,
                                               false,
                                               &batch)) {
            return false;
        }
        if (!arm_send_batch(&batch)) {
            return false;
        }
        if (!arm_wait_ack(addr, 0x9AU)) {
            return false;
        }
        log_info("core.dev.arm",
                 "arm home joint %u/%u triggered",
                 (unsigned int)addr,
                 (unsigned int)ARM_JOINT_COUNT);
        if (settle_ms > 0) {
            usleep((useconds_t)settle_ms * 1000U);
        }
        if (!arm_move_single_joint_absolute(cfg, target_deg)) {
            log_warn("core.dev.arm",
                     "arm post-home move joint %u -> %.2fdeg failed",
                     (unsigned int)addr,
                     target_deg);
            return false;
        }
        log_info("core.dev.arm",
                 "arm post-home move joint %u -> %.2fdeg",
                 (unsigned int)addr,
                 target_deg);
        if (post_home_move_delay_ms > 0 && i + 1U < ARM_JOINT_COUNT) {
            usleep((useconds_t)post_home_move_delay_ms * 1000U);
        }
    }

    log_info("core.dev.arm", "arm home sequence completed with post-home safe pose updates");
    return true;
}

static bool arm_home_single_joint(int joint_index) {
    ZdtArmCanBatch batch;
    const uint8_t home_mode = arm_get_home_mode();
    const uint8_t addr = (uint8_t)joint_index;

    if (joint_index < 1 || joint_index > (int)ARM_JOINT_COUNT) {
        log_warn("core.dev.arm", "invalid home joint index=%d", joint_index);
        return false;
    }

    arm_drain_rx();
    if (!arm_enable_joint(addr)) {
        log_warn("core.dev.arm",
                 "arm home joint %u enable ack missed, continue with home trigger",
                 (unsigned int)addr);
    }
    if (!arm_write_zero_params(addr)) {
        log_warn("core.dev.arm",
                 "arm home joint %u zero params write failed",
                 (unsigned int)addr);
        return false;
    }
    if (!proto_zdt_arm_encode_trigger_home(addr,
                                           home_mode,
                                           false,
                                           &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (!arm_wait_ack(addr, 0x9AU)) {
        return false;
    }

    /* Wait for the motor to complete stall-detection homing before returning.
     * Without this delay the caller immediately sends a move-to-safe command
     * while the motor is still travelling to the hard-stop. */
    const int settle_ms = arm_get_home_settle_ms();
    if (settle_ms > 0) {
        usleep((useconds_t)settle_ms * 1000U);
    }

    log_info("core.dev.arm", "arm home single joint %u triggered", (unsigned int)addr);
    return true;
}

static bool arm_stop_all_joints(void) {
    ZdtArmCanBatch batch;
    size_t i;
    bool any_responded = false;

    arm_drain_rx();
    /* Use sync=false so each motor stops IMMEDIATELY upon receiving the
     * frame. The previous sync=true version queued the stop in the motor
     * and required a sync_start broadcast to trigger it; that left the
     * motor in a queued/synced state from which subsequent home (0x9A)
     * commands are rejected with 0xE2. Direct stops avoid that pitfall. */
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        const uint8_t addr = (uint8_t)(i + 1U);
        if (!proto_zdt_arm_encode_stop(addr, false, &batch)) {
            continue;
        }
        if (!arm_send_batch(&batch)) {
            continue;
        }
        /* Probe whether ANY response came back. The ZDT motor may answer
         * with status=OK (it just stopped) OR status=REJECTED (it was
         * already idle/stopped). Both cases mean the motor is on the bus
         * and now in a stopped state - treat both as success. Only a true
         * timeout (no frame at all) means the motor is not connected. */
        ZdtArmResponse resp;
        uint8_t resp_addr = 0U;
        bool got_response = false;
        const int ack_timeout_ms =
            arm_getenv_int("UAV_ARM_ACK_TIMEOUT_MS", ARM_DEFAULT_ACK_TIMEOUT_MS);
        while (arm_receive_response_once(&resp_addr, &resp, ack_timeout_ms)) {
            if (resp_addr != 0U && resp_addr != addr) {
                continue;
            }
            if (resp.cmd != 0xFEU) {
                continue;
            }
            got_response = true;
            break;
        }
        if (!got_response) {
            log_warn("core.dev.arm",
                     "joint %u stop no response (not connected?)",
                     (unsigned)addr);
            continue;
        }
        any_responded = true;
        if (resp.type == ZDT_ARM_RESP_ACK_REJECTED) {
            log_info("core.dev.arm",
                     "joint %u stop rejected status=0x%02X (already idle)",
                     (unsigned)addr, (unsigned)resp.status);
        } else {
            log_info("core.dev.arm", "joint %u stopped", (unsigned)addr);
        }
    }

    if (!any_responded) {
        log_error("core.dev.arm", "emergency stop: no joints responded");
        return false;
    }

    return true;
}

static bool arm_stop_single_joint(int joint_index) {
    ZdtArmCanBatch batch;
    const uint8_t addr = (uint8_t)joint_index;

    if (joint_index < 1 || joint_index > (int)ARM_JOINT_COUNT) {
        log_warn("core.dev.arm", "invalid stop joint index=%d", joint_index);
        return false;
    }

    arm_drain_rx();
    if (!proto_zdt_arm_encode_stop(addr, false, &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (!arm_wait_ack(addr, 0xFEU)) {
        return false;
    }

    log_info("core.dev.arm", "arm stop single joint %u ok", (unsigned int)addr);
    return true;
}

static bool arm_solve_position_ik(double x_mm,
                                  double y_mm,
                                  double z_mm,
                                  double out_joints_deg[ARM_JOINT_COUNT]) {
    const double link3_eff_mm = sqrt(ARM_LINK3_A_MM * ARM_LINK3_A_MM +
                                     ARM_LINK4_D_MM * ARM_LINK4_D_MM);
    const double link3_beta_rad = atan2(ARM_LINK4_D_MM, ARM_LINK3_A_MM);
    double radial_mm;
    double wrist_r_mm;
    double wrist_z_mm;
    double dist_sq;
    double cos_phi;
    double phi_rad;
    double shoulder_rad;
    double q1_rad;
    double q2_rad;
    double q3_rad;

    if (out_joints_deg == NULL) {
        return false;
    }

    radial_mm = hypot(x_mm, y_mm);
    wrist_r_mm = radial_mm - ARM_BASE_OFFSET_MM - ARM_TOOL_OFFSET_MM;
    wrist_z_mm = z_mm - ARM_BASE_HEIGHT_MM;

    if (wrist_r_mm < 1.0) {
        wrist_r_mm = 1.0;
    }

    dist_sq = wrist_r_mm * wrist_r_mm + wrist_z_mm * wrist_z_mm;
    cos_phi = (dist_sq - ARM_LINK2_MM * ARM_LINK2_MM - link3_eff_mm * link3_eff_mm) /
              (2.0 * ARM_LINK2_MM * link3_eff_mm);
    if (cos_phi < -1.0 || cos_phi > 1.0) {
        log_warn("core.dev.arm",
                 "xyz target unreachable x=%.1f y=%.1f z=%.1f",
                 x_mm,
                 y_mm,
                 z_mm);
        return false;
    }

    if (cos_phi > 1.0) {
        cos_phi = 1.0;
    }
    if (cos_phi < -1.0) {
        cos_phi = -1.0;
    }

    phi_rad = atan2(sqrt(1.0 - cos_phi * cos_phi), cos_phi);
    shoulder_rad = atan2(wrist_z_mm, wrist_r_mm) -
                   atan2(link3_eff_mm * sin(phi_rad),
                         ARM_LINK2_MM + link3_eff_mm * cos(phi_rad));
    q1_rad = atan2(y_mm, x_mm);
    q2_rad = shoulder_rad;
    q3_rad = phi_rad - link3_beta_rad;

    out_joints_deg[0] = arm_rad_to_deg(q1_rad);
    out_joints_deg[1] = arm_rad_to_deg(q2_rad);
    out_joints_deg[2] = arm_rad_to_deg(q3_rad);

    /* Keep a simple neutral wrist posture until full orientation IK is added. */
    out_joints_deg[3] = 0.0;
    out_joints_deg[4] = 0.0;
    out_joints_deg[5] = 0.0;
    return true;
}

bool arm_move(const char *pose) {
    static const double k_home_joints[ARM_JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const double k_approach_joints[ARM_JOINT_COUNT] = {0.0, 55.0, -30.0, 0.0, 0.0, 0.0};
    static const double k_lift_joints[ARM_JOINT_COUNT] = {0.0, 35.0, -20.0, 0.0, 0.0, 0.0};

    if (pose == NULL) {
        return false;
    }

    if (strcmp(pose, "home") == 0) {
        return arm_home_all_joints();
    }
    if (strcmp(pose, "approach") == 0) {
        return arm_move_joints_deg_impl(k_approach_joints);
    }
    if (strcmp(pose, "lift") == 0) {
        return arm_move_joints_deg_impl(k_lift_joints);
    }
    if (strcmp(pose, "zero") == 0) {
        return arm_move_joints_deg_impl(k_home_joints);
    }

    log_warn("core.dev.arm", "unknown pose=%s", pose);
    return false;
}

bool arm_move_to_xyz(float x_mm, float y_mm, float z_mm) {
    double joints_deg[ARM_JOINT_COUNT];

    if (!arm_solve_position_ik((double)x_mm, (double)y_mm, (double)z_mm, joints_deg)) {
        return false;
    }
    if (!arm_move_joints_deg_impl(joints_deg)) {
        return false;
    }

    log_info("core.dev.arm",
             "arm_move_to_xyz(x=%.1f, y=%.1f, z=%.1f) -> joints=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
             (double)x_mm,
             (double)y_mm,
             (double)z_mm,
             joints_deg[0],
             joints_deg[1],
             joints_deg[2],
             joints_deg[3],
             joints_deg[4],
             joints_deg[5]);
    return true;
}

bool arm_move_to_pose6d(float x_mm,
                        float y_mm,
                        float z_mm,
                        float roll_deg,
                        float pitch_deg,
                        float yaw_deg) {
    double joints_deg[ARM_JOINT_COUNT];
    ArmJointConfig j4;
    ArmJointConfig j5;
    ArmJointConfig j6;

    if (!arm_solve_position_ik((double)x_mm, (double)y_mm, (double)z_mm, joints_deg)) {
        return false;
    }

    j4 = arm_joint_config(3U);
    j5 = arm_joint_config(4U);
    j6 = arm_joint_config(5U);

    joints_deg[3] = arm_deg_clamp((double)roll_deg, j4.min_deg, j4.max_deg);
    joints_deg[4] = arm_deg_clamp((double)pitch_deg, j5.min_deg, j5.max_deg);
    joints_deg[5] = arm_deg_clamp((double)yaw_deg, j6.min_deg, j6.max_deg);

    if (!arm_move_joints_deg_impl(joints_deg)) {
        return false;
    }

    log_info("core.dev.arm",
             "arm_move_to_pose6d(x=%.1f, y=%.1f, z=%.1f, roll=%.1f, pitch=%.1f, yaw=%.1f)",
             (double)x_mm,
             (double)y_mm,
             (double)z_mm,
             (double)roll_deg,
             (double)pitch_deg,
             (double)yaw_deg);
    return true;
}

bool arm_move_joint_deg(int joint_index, float target_deg) {
    ArmJointConfig cfg;
    ZdtArmCanBatch batch;

    if (joint_index < 1 || joint_index > (int)ARM_JOINT_COUNT) {
        log_warn("core.dev.arm", "invalid joint index=%d", joint_index);
        return false;
    }

    arm_drain_rx();
    cfg = arm_joint_config((size_t)(joint_index - 1));
    if (!arm_plan_joint_absolute(cfg, (double)target_deg, false, &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (arm_should_wait_reached() && !arm_wait_position_reached(cfg.addr)) {
        return false;
    }

    log_info("core.dev.arm",
             "arm_move_joint_deg(joint=%d, target=%.1f)",
             joint_index,
             (double)target_deg);
    return true;
}

bool arm_move_joints_deg(const float joints_deg[6]) {
    double targets[ARM_JOINT_COUNT];
    size_t i;

    if (joints_deg == NULL) {
        return false;
    }
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        targets[i] = (double)joints_deg[i];
    }
    return arm_move_joints_deg_impl(targets);
}

bool arm_home(void) {
    return arm_home_all_joints();
}

bool arm_home_joint(int joint_index) {
    return arm_home_single_joint(joint_index);
}

bool arm_set_zero_params(int joint_index) {
    if (joint_index < 1 || joint_index > (int)ARM_JOINT_COUNT) {
        log_warn("core.dev.arm", "invalid zero params joint index=%d", joint_index);
        return false;
    }

    arm_drain_rx();
    if (!arm_enable_joint((uint8_t)joint_index)) {
        log_warn("core.dev.arm",
                 "arm zero params joint %d enable ack missed, continue with write",
                 joint_index);
    }
    return arm_write_zero_params((uint8_t)joint_index);
}

bool arm_stop_joint(int joint_index) {
    return arm_stop_single_joint(joint_index);
}

bool arm_stop(void) {
    /* Clear the estop pipe/flag so the stop frames below can wait for ack
     * normally. The flag was likely set by the watchdog that triggered us. */
    arm_clear_estop();
    log_warn("core.dev.arm", "arm_stop: executing emergency stop on all joints");
    return arm_stop_all_joints();
}
