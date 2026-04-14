/* task_face_track.c
 *
 * Continuously nudges joint 1 to keep the detected face centered in the
 * camera frame. Input: face_tracker.py publishes class_id=100 detections
 * on the standard npu result socket. Output: arm_move_joint_deg() via
 * arm_rpc_proxy → proc_arm → CAN.
 *
 * Controller is a simple proportional step clamped to max_step_deg per
 * tick (≈200 ms scheduler period). Small deadzone avoids jitter.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "app_face_track.h"
#include "dev.h"
#include "log.h"
#include "npu_detect.h"

#define FT_DEFAULT_JOINT         1
#define FT_DEFAULT_FOV_H_DEG     70.0F    /* RealSense D435 horizontal FOV */
#define FT_DEFAULT_DEADZONE      0.06F    /* 6% of frame width */
#define FT_DEFAULT_MAX_STEP_DEG  2.5F
#define FT_IDLE_TIMEOUT_TICKS    25       /* ≈5 s without a face → idle */

/* Joint 1 is wired so that moving positive rotates the base clockwise from
 * the robot's POV. The camera's +X (right) maps to the SAME direction for
 * an eye-in-hand setup on a front-facing camera. If you find the arm
 * tracks the WRONG direction, flip this sign. */
#define FT_J1_DIRECTION_SIGN    (+1.0F)

/* Joint 1 min/max from stall-homing (0..360°). Clamp so the proportional
 * step cannot drive us out of travel. */
#define FT_J1_MIN_DEG            5.0F
#define FT_J1_MAX_DEG            355.0F

void app_face_track_start(FaceTrackTask *task) {
    if (task == NULL) return;
    task->state            = FT_INIT;
    task->ticks_in_state   = 0;
    task->ticks_since_seen = 0;
    task->current_j1_deg   = 180.0F;       /* post-home safe center */
    task->joint_index      = FT_DEFAULT_JOINT;
    task->fov_h_deg        = FT_DEFAULT_FOV_H_DEG;
    task->deadzone_frac    = FT_DEFAULT_DEADZONE;
    task->max_step_deg     = FT_DEFAULT_MAX_STEP_DEG;
    log_info("app.face_track",
             "start: joint=%d fov=%.1f dz=%.2f maxstep=%.1f",
             task->joint_index,
             (double)task->fov_h_deg,
             (double)task->deadzone_frac,
             (double)task->max_step_deg);
}

bool app_face_track_done(const FaceTrackTask *task) {
    return task != NULL && task->state == FT_DONE;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

bool app_face_track_step(FaceTrackTask *task,
                         const DeviceRegistry *reg,
                         char *err,
                         int err_len) {
    (void)reg;
    if (task == NULL) {
        snprintf(err, (size_t)err_len, "task null");
        return false;
    }
    task->ticks_in_state++;

    if (task->state == FT_INIT) {
        task->state = FT_TRACKING;
        task->ticks_in_state = 0;
    }

    float cx_px = 0.0F, cy_px = 0.0F;
    float w = 640.0F, h = 480.0F;
    const bool seen = npu_read_latest_face(&cx_px, &cy_px, &w, &h);

    if (!seen) {
        task->ticks_since_seen++;
        if (task->ticks_since_seen > FT_IDLE_TIMEOUT_TICKS) {
            if (task->state != FT_IDLE) {
                log_info("app.face_track", "no face for %d ticks — idle",
                         task->ticks_since_seen);
                task->state = FT_IDLE;
            }
        }
        return true;  /* keep stepping — not an error */
    }

    task->ticks_since_seen = 0;
    if (task->state == FT_IDLE) {
        log_info("app.face_track", "face reacquired");
        task->state = FT_TRACKING;
    }

    /* Normalized horizontal offset: -1 (left edge) .. +1 (right edge). */
    const float half_w = (w > 1.0F) ? (w * 0.5F) : 320.0F;
    const float offset = (cx_px - half_w) / half_w;

    /* Deadzone → no move. */
    if (fabsf(offset) < task->deadzone_frac) {
        return true;
    }

    /* Proportional: map offset*FOV/2 to a J1 step, clamped. */
    const float raw_step = offset * (task->fov_h_deg * 0.5F) * 0.3F;
    float step = clampf(raw_step, -task->max_step_deg, task->max_step_deg);
    step *= FT_J1_DIRECTION_SIGN;

    float new_j1 = clampf(task->current_j1_deg + step, FT_J1_MIN_DEG, FT_J1_MAX_DEG);
    if (fabsf(new_j1 - task->current_j1_deg) < 0.05F) {
        return true;  /* too small to bother moving */
    }

    if (!arm_move_joint_deg(task->joint_index, new_j1)) {
        /* Non-fatal: proc_arm might be briefly busy. Just skip this tick. */
        log_warn("app.face_track",
                 "arm_move_joint_deg(%d,%.2f) failed, retrying",
                 task->joint_index, (double)new_j1);
        return true;
    }
    task->current_j1_deg = new_j1;
    return true;
}
