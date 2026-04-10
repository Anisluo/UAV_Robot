#ifndef UAV_APP_ARM_GRASP_H
#define UAV_APP_ARM_GRASP_H

#include <stdbool.h>
#include "device_registry.h"
#include "abi/msg_types.h"

/*
 * 3D/6D vision-guided arm grasping FSM.
 *
 * State flow:
 *   PRECHECK → SAFE_POSE → DETECT → CONFIRM → PRE_APPROACH
 *   → GRIPPER_OPEN → DESCEND → GRASP → LIFT → RETREAT → DONE
 *
 * On any hard error the FSM enters FAIL and returns false from arm_grasp_step().
 * Soft errors (descend/grasp failure) trigger up to AG_GRASP_RETRY_MAX retries
 * by rewinding to PRE_APPROACH / GRIPPER_OPEN respectively.
 */

/* Timeout in scheduler ticks (~200 ms each) */
#define AG_DETECT_TIMEOUT_TICKS    100  /* 20 s to get first in-workspace detection  */
#define AG_CONFIRM_TIMEOUT_TICKS    30  /* 6 s to accumulate confirmation frames      */

/* How many consecutive in-workspace detections to average before committing */
#define AG_CONFIRM_FRAMES           3

/* Retry budget for the descend-and-grasp phase */
#define AG_GRASP_RETRY_MAX          2

/* Pre-grasp approach: arm moves to target XY but this far above Z (mm) */
#define AG_PREAPPROACH_Z_OFFSET_MM  80.0f

/* Post-grasp lift target Z (mm above base) */
#define AG_LIFT_Z_MM               350.0f

/* Workspace hard limits used by the bounds check */
#define AG_WS_Z_MIN_MM              30.0f   /* lowest valid target Z   */
#define AG_WS_Z_MAX_MM             650.0f   /* highest valid target Z  */
#define AG_WS_R_MIN_MM              80.0f   /* minimum XY radial reach */
#define AG_WS_R_MAX_MM             440.0f   /* maximum XY radial reach */

typedef enum {
    AG_IDLE = 0,
    AG_PRECHECK,
    AG_SAFE_POSE,
    AG_DETECT,
    AG_CONFIRM,
    AG_PRE_APPROACH,
    AG_GRIPPER_OPEN,
    AG_DESCEND,
    AG_GRASP,
    AG_LIFT,
    AG_RETREAT,
    AG_DONE,
    AG_FAIL
} ArmGraspState;

typedef struct {
    ArmGraspState state;
    int           ticks_in_state;
    UavGraspMode  grasp_mode;

    /* Confirmed averaged target pose */
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float target_roll_deg;
    float target_pitch_deg;
    float target_yaw_deg;
    int   has_rpy;              /* 1 if orientation data is valid */

    /* Running accumulator for multi-frame confirmation */
    int   confirm_count;
    float confirm_sum_x;
    float confirm_sum_y;
    float confirm_sum_z;
    float confirm_sum_roll;
    float confirm_sum_pitch;
    float confirm_sum_yaw;

    /* Retry counter (resets on arm_grasp_start) */
    int   retry_count;
} ArmGraspTask;

void arm_grasp_start(ArmGraspTask *task, UavGraspMode grasp_mode);
bool arm_grasp_step(ArmGraspTask *task, const DeviceRegistry *reg, char *err, int err_len);
bool arm_grasp_done(const ArmGraspTask *task);

/* Human-readable state name (for logging/status export) */
const char *arm_grasp_state_name(ArmGraspState state);

#endif /* UAV_APP_ARM_GRASP_H */
