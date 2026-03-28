#include <stdio.h>
#include <string.h>
#include "app_battery_pick.h"
#include "dev.h"
#include "log.h"
#include "npu_detect.h"

void app_battery_pick_start(BatteryPickTask *task, UavGraspMode grasp_mode) {
    task->state        = BP_PRECHECK;
    task->ticks_in_state = 0;
    task->detected     = 0;
    task->grasp_mode   = grasp_mode;
    task->target_x_mm  = 0.0f;
    task->target_y_mm  = 0.0f;
    task->target_z_mm  = 0.0f;
    task->target_roll_deg = 0.0f;
    task->target_pitch_deg = 0.0f;
    task->target_yaw_deg = 0.0f;
    task->has_rpy = 0;
    log_info("app.battery_pick", "start FSM grasp_mode=%s",
             grasp_mode == UAV_GRASP_MODE_6D ? "6d" : "3d");
}

bool app_battery_pick_done(const BatteryPickTask *task) {
    return task->state == BP_DONE;
}

static void next_state(BatteryPickTask *task, BatteryPickState st) {
    task->state = st;
    task->ticks_in_state = 0;
}

bool app_battery_pick_step(BatteryPickTask *task, const DeviceRegistry *reg, char *err, int err_len) {
    (void)err_len;
    task->ticks_in_state++;

    switch (task->state) {
        case BP_PRECHECK:
            if (!device_registry_check_ready(reg)) {
                strcpy(err, "device offline");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_FIXTURE_LOCK);
            return true;

        case BP_FIXTURE_LOCK:
            if (!platform_lock(true)) {
                strcpy(err, "platform lock failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_DETECT);
            return true;

        case BP_DETECT: {
            NpuGraspPose pose;
            if (npu_read_latest_grasp_pose(task->grasp_mode, &pose)) {
                task->target_x_mm = pose.x_mm;
                task->target_y_mm = pose.y_mm;
                task->target_z_mm = pose.z_mm;
                task->target_roll_deg = pose.roll_deg;
                task->target_pitch_deg = pose.pitch_deg;
                task->target_yaw_deg = pose.yaw_deg;
                task->has_rpy = pose.has_rpy ? 1 : 0;
                task->detected    = 1;
                log_info("app.battery_pick",
                         "battery detected mode=%s pos=(%.1f, %.1f, %.1f) rpy=(%.1f, %.1f, %.1f) has_rpy=%d",
                         task->grasp_mode == UAV_GRASP_MODE_6D ? "6d" : "3d",
                         pose.x_mm, pose.y_mm, pose.z_mm,
                         pose.roll_deg, pose.pitch_deg, pose.yaw_deg,
                         pose.has_rpy ? 1 : 0);
                next_state(task, BP_APPROACH);
            } else if (task->ticks_in_state >= BP_DETECT_TIMEOUT_TICKS) {
                strcpy(err, "detect timeout: no battery found");
                next_state(task, BP_FAIL);
                return false;
            }
            /* Stay in BP_DETECT until detection arrives or timeout. */
            return true;
        }

        case BP_APPROACH:
            if (task->detected) {
                if (task->grasp_mode == UAV_GRASP_MODE_6D) {
                    if (!task->has_rpy) {
                        strcpy(err, "6d pose missing orientation");
                        next_state(task, BP_FAIL);
                        return false;
                    }
                    if (!arm_move_to_pose6d(task->target_x_mm,
                                            task->target_y_mm,
                                            task->target_z_mm,
                                            task->target_roll_deg,
                                            task->target_pitch_deg,
                                            task->target_yaw_deg)) {
                        strcpy(err, "arm approach (6d) failed");
                        next_state(task, BP_FAIL);
                        return false;
                    }
                } else {
                    if (!arm_move_to_xyz(task->target_x_mm,
                                         task->target_y_mm,
                                         task->target_z_mm)) {
                        strcpy(err, "arm approach (3d) failed");
                        next_state(task, BP_FAIL);
                        return false;
                    }
                }
            } else {
                /* Fallback to preset approach pose */
                if (!arm_move("approach")) {
                    strcpy(err, "arm approach failed");
                    next_state(task, BP_FAIL);
                    return false;
                }
            }
            next_state(task, BP_GRIPPER_OPEN);
            return true;

        case BP_GRIPPER_OPEN:
            if (!gripper_open(true)) {
                strcpy(err, "gripper open failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_ALIGN);
            return true;

        case BP_ALIGN:
            if (!car_set_velocity(0, 0)) {
                strcpy(err, "car stop failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_GRASP);
            return true;

        case BP_GRASP:
            if (!gripper_open(false)) {
                strcpy(err, "grasp failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_LIFT);
            return true;

        case BP_LIFT:
            if (!arm_move("lift")) {
                strcpy(err, "lift failed");
                next_state(task, BP_FAIL);
                return false;
            }
            next_state(task, BP_DONE);
            return true;

        case BP_DONE:
            return true;

        case BP_FAIL:
        default:
            return false;
    }
}
