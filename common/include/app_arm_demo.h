#ifndef UAV_APP_ARM_DEMO_H
#define UAV_APP_ARM_DEMO_H

#include <stdbool.h>
#include "device_registry.h"

/* Simple arm motion demo task. Used to validate the full task scheduling
 * pipeline (Tab4 -> proc_gateway -> uav_robotd -> task FSM -> arm_rpc_proxy
 * -> proc_arm -> CAN) without depending on realsense/npu/gripper hardware.
 *
 * Plays a 3-step sweep on joint 1: 0deg -> +30deg -> -30deg -> 0deg.
 * Each step is one FSM tick (~200ms scheduler period). Interruptible by
 * estop because the FSM checks SystemState.emergency_stop between steps.
 *
 * On hosts where joint 1 is the only motor connected, this exercises the
 * full chain end-to-end while the rest of the robot stays idle. */

typedef enum {
    AD_INIT = 0,
    AD_MOVE_POS,
    AD_DWELL_POS,
    AD_MOVE_NEG,
    AD_DWELL_NEG,
    AD_MOVE_ZERO,
    AD_DONE,
    AD_FAIL
} ArmDemoState;

typedef struct {
    ArmDemoState state;
    int          ticks_in_state;
    int          joint_index;     /* default: 1 */
    float        sweep_deg;       /* default: 30 */
    int          dwell_ticks;     /* default: 5 ticks (~1 s) */
} ArmDemoTask;

void app_arm_demo_start(ArmDemoTask *task);
bool app_arm_demo_step(ArmDemoTask *task,
                       const DeviceRegistry *reg,
                       char *err,
                       int err_len);
bool app_arm_demo_done(const ArmDemoTask *task);

#endif
