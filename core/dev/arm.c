#include "dev.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "can_socketcan.h"
#include "log.h"
#include "proto_zdt_arm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM_JOINT_COUNT 6U
#define ARM_DEFAULT_RPM 300U
#define ARM_DEFAULT_ACC 20U
#define ARM_HOME_MODE_NEAREST 0U
#define ARM_MOTOR_PULSES_PER_REV 3200.0

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

static bool arm_send_batch(const ZdtArmCanBatch *batch) {
    size_t i;

    if (batch == NULL) {
        return false;
    }
    for (i = 0; i < batch->count; ++i) {
        if (!can_socketcan_send(batch->frames[i].can_id,
                                batch->frames[i].is_extended_id,
                                batch->frames[i].data,
                                batch->frames[i].len)) {
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
    return arm_send_batch(&batch);
}

static ArmJointConfig arm_joint_config(size_t joint_index) {
    static const double k_default_ratios[ARM_JOINT_COUNT] = {25.0, 20.0, 25.0, 10.0, 1.0, 1.0};
    static const double k_default_min_deg[ARM_JOINT_COUNT] = {-180.0, 0.0, -80.0, -30.0, -110.0, -30.0};
    static const double k_default_max_deg[ARM_JOINT_COUNT] = {160.0, 180.0, 83.0, 305.0, 110.0, 305.0};
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
    ArmJointConfig cfg;

    cfg.addr = (uint8_t)(joint_index + 1U);
    cfg.ratio = arm_getenv_double(k_ratio_env[joint_index], k_default_ratios[joint_index]);
    cfg.min_deg = k_default_min_deg[joint_index];
    cfg.max_deg = k_default_max_deg[joint_index];
    cfg.dir_sign = arm_getenv_int(k_dir_env[joint_index], k_default_dirs[joint_index]);
    cfg.zero_offset_pulses = arm_getenv_int(k_zero_env[joint_index], 0);
    if (cfg.dir_sign >= 0) {
        cfg.dir_sign = 1;
    } else {
        cfg.dir_sign = -1;
    }
    return cfg;
}

static bool arm_plan_joint_absolute(ArmJointConfig cfg,
                                    double target_deg,
                                    bool sync,
                                    ZdtArmCanBatch *out_batch) {
    double pulses_per_degree = (ARM_MOTOR_PULSES_PER_REV * cfg.ratio) / 360.0;
    int signed_pulses;
    bool ccw;

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
    if (!arm_enable_joint(cfg.addr)) {
        return false;
    }

    signed_pulses = arm_round_to_int(target_deg * pulses_per_degree * (double)cfg.dir_sign) +
                    cfg.zero_offset_pulses;
    ccw = signed_pulses < 0;
    if (signed_pulses < 0) {
        signed_pulses = -signed_pulses;
    }

    return proto_zdt_arm_encode_position(cfg.addr,
                                         ccw,
                                         ARM_DEFAULT_RPM,
                                         ARM_DEFAULT_ACC,
                                         (uint32_t)signed_pulses,
                                         true,
                                         sync,
                                         out_batch);
}

static bool arm_move_joints_deg(const double joints_deg[ARM_JOINT_COUNT]) {
    ArmJointTarget targets[ARM_JOINT_COUNT];
    ZdtArmCanBatch batch;
    size_t i;

    if (joints_deg == NULL) {
        return false;
    }

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
    return arm_send_batch(&batch);
}

static bool arm_home_all_joints(void) {
    ZdtArmCanBatch batch;
    size_t i;

    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (!arm_enable_joint((uint8_t)(i + 1U))) {
            return false;
        }
        if (!proto_zdt_arm_encode_trigger_home((uint8_t)(i + 1U),
                                               ARM_HOME_MODE_NEAREST,
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
        return arm_move_joints_deg(k_approach_joints);
    }
    if (strcmp(pose, "lift") == 0) {
        return arm_move_joints_deg(k_lift_joints);
    }
    if (strcmp(pose, "zero") == 0) {
        return arm_move_joints_deg(k_home_joints);
    }

    log_warn("core.dev.arm", "unknown pose=%s", pose);
    return false;
}

bool arm_move_to_xyz(float x_mm, float y_mm, float z_mm) {
    double joints_deg[ARM_JOINT_COUNT];

    if (!arm_solve_position_ik((double)x_mm, (double)y_mm, (double)z_mm, joints_deg)) {
        return false;
    }
    if (!arm_move_joints_deg(joints_deg)) {
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
