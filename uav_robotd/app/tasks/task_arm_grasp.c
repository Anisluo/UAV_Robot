#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_arm_grasp.h"
#include "dev.h"
#include "log.h"
#include "npu_detect.h"

/* ── helpers ──────────────────────────────────────────────────────────── */

static void ag_next(ArmGraspTask *t, ArmGraspState st)
{
    t->state = st;
    t->ticks_in_state = 0;
}

/*
 * Validate that a target position lies inside the arm's reachable workspace.
 * We check Z range and XY radial distance.  Actual joint-limit clipping is
 * handled by the IK layer; this is a fast pre-filter to reject clearly bogus
 * NPU outputs (e.g. depth projection artefacts at z ≈ 0).
 */
static bool ag_in_workspace(float x_mm, float y_mm, float z_mm)
{
    float r_mm = sqrtf(x_mm * x_mm + y_mm * y_mm);

    if (z_mm < AG_WS_Z_MIN_MM || z_mm > AG_WS_Z_MAX_MM) {
        log_warn("app.arm_grasp",
                 "pose z=%.1f out of workspace z=[%.0f, %.0f] – skip",
                 (double)z_mm,
                 (double)AG_WS_Z_MIN_MM,
                 (double)AG_WS_Z_MAX_MM);
        return false;
    }
    if (r_mm < AG_WS_R_MIN_MM || r_mm > AG_WS_R_MAX_MM) {
        log_warn("app.arm_grasp",
                 "pose r=%.1f out of workspace r=[%.0f, %.0f] – skip",
                 (double)r_mm,
                 (double)AG_WS_R_MIN_MM,
                 (double)AG_WS_R_MAX_MM);
        return false;
    }
    return true;
}

/* Reset the detection accumulator (called on entry to DETECT/SAFE_POSE). */
static void ag_reset_confirm(ArmGraspTask *t)
{
    t->confirm_count    = 0;
    t->confirm_sum_x    = 0.0f;
    t->confirm_sum_y    = 0.0f;
    t->confirm_sum_z    = 0.0f;
    t->confirm_sum_roll = 0.0f;
    t->confirm_sum_pitch = 0.0f;
    t->confirm_sum_yaw  = 0.0f;
}

/* Accumulate one valid detection frame. */
static void ag_accumulate(ArmGraspTask *t, const NpuGraspPose *p)
{
    t->confirm_count++;
    t->confirm_sum_x     += p->x_mm;
    t->confirm_sum_y     += p->y_mm;
    t->confirm_sum_z     += p->z_mm;
    t->confirm_sum_roll  += p->roll_deg;
    t->confirm_sum_pitch += p->pitch_deg;
    t->confirm_sum_yaw   += p->yaw_deg;
    if (p->has_rpy) {
        t->has_rpy = 1;
    }
}

/* Commit the averaged target from the accumulator. */
static void ag_commit_target(ArmGraspTask *t)
{
    float n = (float)t->confirm_count;
    t->target_x_mm      = t->confirm_sum_x     / n;
    t->target_y_mm      = t->confirm_sum_y     / n;
    t->target_z_mm      = t->confirm_sum_z     / n;
    t->target_roll_deg  = t->confirm_sum_roll  / n;
    t->target_pitch_deg = t->confirm_sum_pitch / n;
    t->target_yaw_deg   = t->confirm_sum_yaw   / n;
    log_info("app.arm_grasp",
             "target confirmed (n=%d) pos=(%.1f, %.1f, %.1f) rpy=(%.1f, %.1f, %.1f) has_rpy=%d",
             t->confirm_count,
             (double)t->target_x_mm,
             (double)t->target_y_mm,
             (double)t->target_z_mm,
             (double)t->target_roll_deg,
             (double)t->target_pitch_deg,
             (double)t->target_yaw_deg,
             t->has_rpy);
}

/* Move arm to pre-grasp position (above target by AG_PREAPPROACH_Z_OFFSET_MM). */
static bool ag_move_pre_approach(const ArmGraspTask *t)
{
    float approach_z = t->target_z_mm + AG_PREAPPROACH_Z_OFFSET_MM;
    if (t->grasp_mode == UAV_GRASP_MODE_6D && t->has_rpy) {
        return arm_move_to_pose6d(t->target_x_mm,
                                  t->target_y_mm,
                                  approach_z,
                                  t->target_roll_deg,
                                  t->target_pitch_deg,
                                  t->target_yaw_deg);
    }
    return arm_move_to_xyz(t->target_x_mm, t->target_y_mm, approach_z);
}

/* Move arm down to exact grasp position. */
static bool ag_move_descend(const ArmGraspTask *t)
{
    if (t->grasp_mode == UAV_GRASP_MODE_6D && t->has_rpy) {
        return arm_move_to_pose6d(t->target_x_mm,
                                  t->target_y_mm,
                                  t->target_z_mm,
                                  t->target_roll_deg,
                                  t->target_pitch_deg,
                                  t->target_yaw_deg);
    }
    return arm_move_to_xyz(t->target_x_mm, t->target_y_mm, t->target_z_mm);
}

/* ── public API ───────────────────────────────────────────────────────── */

void arm_grasp_start(ArmGraspTask *task, UavGraspMode grasp_mode)
{
    memset(task, 0, sizeof(*task));
    task->state      = AG_PRECHECK;
    task->grasp_mode = grasp_mode;
    log_info("app.arm_grasp", "FSM start mode=%s",
             grasp_mode == UAV_GRASP_MODE_6D ? "6d" : "3d");
}

bool arm_grasp_done(const ArmGraspTask *task)
{
    return task->state == AG_DONE;
}

const char *arm_grasp_state_name(ArmGraspState state)
{
    switch (state) {
        case AG_IDLE:         return "IDLE";
        case AG_PRECHECK:     return "PRECHECK";
        case AG_SAFE_POSE:    return "SAFE_POSE";
        case AG_DETECT:       return "DETECT";
        case AG_CONFIRM:      return "CONFIRM";
        case AG_PRE_APPROACH: return "PRE_APPROACH";
        case AG_GRIPPER_OPEN: return "GRIPPER_OPEN";
        case AG_DESCEND:      return "DESCEND";
        case AG_GRASP:        return "GRASP";
        case AG_LIFT:         return "LIFT";
        case AG_RETREAT:      return "RETREAT";
        case AG_DONE:         return "DONE";
        case AG_FAIL:         return "FAIL";
        default:              return "UNKNOWN";
    }
}

/*
 * Advance the FSM by one scheduler tick (~200 ms).
 *
 * Returns true while the task is still in progress (including DONE).
 * Returns false on unrecoverable failure; err is set to a short reason string.
 *
 * The caller must stop stepping after the first false return.
 */
bool arm_grasp_step(ArmGraspTask *task, const DeviceRegistry *reg, char *err, int err_len)
{
    task->ticks_in_state++;

    switch (task->state) {

        /* ── device readiness ─────────────────────────────────────────── */
        case AG_PRECHECK:
            if (!device_registry_check_ready(reg)) {
                snprintf(err, (size_t)err_len, "device offline");
                ag_next(task, AG_FAIL);
                return false;
            }
            ag_next(task, AG_SAFE_POSE);
            return true;

        /* ── move to camera-visible observation pose ──────────────────── */
        case AG_SAFE_POSE:
            if (!arm_move("approach")) {
                snprintf(err, (size_t)err_len, "safe pose failed");
                ag_next(task, AG_FAIL);
                return false;
            }
            ag_reset_confirm(task);
            task->has_rpy = 0;
            ag_next(task, AG_DETECT);
            return true;

        /* ── wait for first in-workspace NPU detection ────────────────── */
        case AG_DETECT: {
            NpuGraspPose pose;
            if (npu_read_latest_grasp_pose(task->grasp_mode, &pose)) {
                if (ag_in_workspace(pose.x_mm, pose.y_mm, pose.z_mm)) {
                    log_info("app.arm_grasp",
                             "first detection pos=(%.1f, %.1f, %.1f) – begin confirm",
                             (double)pose.x_mm,
                             (double)pose.y_mm,
                             (double)pose.z_mm);
                    ag_reset_confirm(task);
                    task->has_rpy = 0;
                    ag_accumulate(task, &pose);
                    ag_next(task, AG_CONFIRM);
                    return true;
                }
            }
            if (task->ticks_in_state >= AG_DETECT_TIMEOUT_TICKS) {
                snprintf(err, (size_t)err_len, "detection timeout: no in-workspace target found");
                ag_next(task, AG_FAIL);
                return false;
            }
            return true;
        }

        /* ── accumulate N frames and average for a stable target ──────── */
        case AG_CONFIRM: {
            NpuGraspPose pose;
            if (npu_read_latest_grasp_pose(task->grasp_mode, &pose)) {
                if (ag_in_workspace(pose.x_mm, pose.y_mm, pose.z_mm)) {
                    ag_accumulate(task, &pose);
                }
            }

            if (task->confirm_count >= AG_CONFIRM_FRAMES) {
                ag_commit_target(task);
                task->retry_count = 0;
                ag_next(task, AG_PRE_APPROACH);
                return true;
            }

            if (task->ticks_in_state >= AG_CONFIRM_TIMEOUT_TICKS) {
                log_warn("app.arm_grasp",
                         "confirm timeout (got %d/%d) – retry detect",
                         task->confirm_count, AG_CONFIRM_FRAMES);
                ag_reset_confirm(task);
                task->has_rpy = 0;
                ag_next(task, AG_DETECT);
                return true;
            }
            return true;
        }

        /* ── approach from above (pre-grasp position) ─────────────────── */
        case AG_PRE_APPROACH:
            if (!ag_move_pre_approach(task)) {
                snprintf(err, (size_t)err_len, "pre-approach failed (IK or CAN error)");
                ag_next(task, AG_FAIL);
                return false;
            }
            ag_next(task, AG_GRIPPER_OPEN);
            return true;

        /* ── open gripper before descending ──────────────────────────── */
        case AG_GRIPPER_OPEN:
            if (!gripper_open(true)) {
                snprintf(err, (size_t)err_len, "gripper open failed");
                ag_next(task, AG_FAIL);
                return false;
            }
            ag_next(task, AG_DESCEND);
            return true;

        /* ── descend to exact grasp position ─────────────────────────── */
        case AG_DESCEND:
            if (!ag_move_descend(task)) {
                if (task->retry_count < AG_GRASP_RETRY_MAX) {
                    task->retry_count++;
                    log_warn("app.arm_grasp",
                             "descend failed – retry %d/%d from pre-approach",
                             task->retry_count, AG_GRASP_RETRY_MAX);
                    ag_next(task, AG_PRE_APPROACH);
                    return true;
                }
                snprintf(err, (size_t)err_len,
                         "descend failed after %d retries", task->retry_count);
                ag_next(task, AG_FAIL);
                return false;
            }
            ag_next(task, AG_GRASP);
            return true;

        /* ── close gripper to grasp ───────────────────────────────────── */
        case AG_GRASP:
            if (!gripper_open(false)) {
                if (task->retry_count < AG_GRASP_RETRY_MAX) {
                    task->retry_count++;
                    log_warn("app.arm_grasp",
                             "grasp failed – retry %d/%d from gripper open",
                             task->retry_count, AG_GRASP_RETRY_MAX);
                    ag_next(task, AG_GRIPPER_OPEN);
                    return true;
                }
                snprintf(err, (size_t)err_len,
                         "grasp close failed after %d retries", task->retry_count);
                ag_next(task, AG_FAIL);
                return false;
            }
            ag_next(task, AG_LIFT);
            return true;

        /* ── lift object straight up ──────────────────────────────────── */
        case AG_LIFT:
            /*
             * Lift vertically above grasp point.  If the IK cannot reach the
             * lift height (e.g. object was very high), fall back to the named
             * "lift" pose which is a safe elbow-up configuration.
             */
            if (!arm_move_to_xyz(task->target_x_mm, task->target_y_mm, AG_LIFT_Z_MM)) {
                log_warn("app.arm_grasp",
                         "vertical lift IK failed (target_z=%.0f) – using fallback lift pose",
                         (double)AG_LIFT_Z_MM);
                if (!arm_move("lift")) {
                    snprintf(err, (size_t)err_len, "lift failed");
                    ag_next(task, AG_FAIL);
                    return false;
                }
            }
            ag_next(task, AG_RETREAT);
            return true;

        /* ── return arm to safe observation position ──────────────────── */
        case AG_RETREAT:
            if (!arm_move("approach")) {
                /* Non-fatal: object is grasped, just warn and complete. */
                log_warn("app.arm_grasp", "retreat to safe pose failed – object is held, marking done");
            }
            ag_next(task, AG_DONE);
            return true;

        /* ── terminal states ──────────────────────────────────────────── */
        case AG_DONE:
            return true;

        case AG_FAIL:
        default:
            return false;

        case AG_IDLE:
            return true;
    }

    /* unreachable */
    return true;
}
