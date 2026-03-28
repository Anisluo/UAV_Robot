#define _DEFAULT_SOURCE

#include "dev.h"

#include <errno.h>
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
#define ARM_HOME_MODE_DEFAULT 2U
#define ARM_MOTOR_PULSES_PER_REV 3200.0
#define ARM_CAN_DEFAULT_IFACE "can4"
#define ARM_DEFAULT_ACK_TIMEOUT_MS 200
#define ARM_DEFAULT_REACHED_TIMEOUT_MS 8000

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

static int g_arm_can_fd = -1;
static char g_arm_can_iface[IF_NAMESIZE] = ARM_CAN_DEFAULT_IFACE;

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
    int rpm = arm_getenv_int("UAV_ARM_RPM", ARM_DEFAULT_RPM);
    if (rpm < 1) {
        rpm = 1;
    }
    if (rpm > 3000) {
        rpm = 3000;
    }
    return (uint16_t)rpm;
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
    struct pollfd pfd;
    struct can_frame frame;
    ssize_t nread;
    ZdtArmResponse resp;
    uint8_t addr = 0U;

    if (out_addr == NULL || out_resp == NULL) {
        return false;
    }
    if (arm_can_init() != 0) {
        return false;
    }

    memset(&pfd, 0, sizeof(pfd));
    pfd.fd = g_arm_can_fd;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout_ms) <= 0) {
        return false;
    }
    if ((pfd.revents & POLLIN) == 0) {
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
        return false;
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
    size_t i;
    const uint8_t home_mode = arm_get_home_mode();
    const int settle_ms = arm_get_home_settle_ms();

    arm_drain_rx();
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        const uint8_t addr = (uint8_t)(i + 1U);
        if (!arm_enable_joint(addr)) {
            log_warn("core.dev.arm",
                     "arm home joint %u enable ack missed, continue with home trigger",
                     (unsigned int)addr);
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
        if (settle_ms > 0 && i + 1U < ARM_JOINT_COUNT) {
            usleep((useconds_t)settle_ms * 1000U);
        }
    }

    log_info("core.dev.arm", "arm home sequence completed, logical zero updated");
    return true;
}

static bool arm_stop_all_joints(void) {
    ZdtArmCanBatch batch;
    size_t i;

    arm_drain_rx();
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (!proto_zdt_arm_encode_stop((uint8_t)(i + 1U), true, &batch)) {
            return false;
        }
        if (!arm_send_batch(&batch)) {
            return false;
        }
        if (!arm_wait_ack((uint8_t)(i + 1U), 0xFEU)) {
            return false;
        }
    }

    if (!proto_zdt_arm_encode_sync_start(&batch)) {
        return false;
    }
    return arm_send_batch(&batch);
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
    ZdtArmCanBatch sync_batch;

    if (joint_index < 1 || joint_index > (int)ARM_JOINT_COUNT) {
        log_warn("core.dev.arm", "invalid joint index=%d", joint_index);
        return false;
    }

    arm_drain_rx();
    cfg = arm_joint_config((size_t)(joint_index - 1));
    if (!arm_plan_joint_absolute(cfg, (double)target_deg, true, &batch)) {
        return false;
    }
    if (!arm_send_batch(&batch)) {
        return false;
    }
    if (!proto_zdt_arm_encode_sync_start(&sync_batch)) {
        return false;
    }
    if (!arm_send_batch(&sync_batch)) {
        return false;
    }
    if (!arm_wait_sync_ack(0U)) {
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

bool arm_stop(void) {
    return arm_stop_all_joints();
}
