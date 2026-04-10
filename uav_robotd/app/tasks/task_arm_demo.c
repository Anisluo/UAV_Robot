/* task_arm_demo.c
 *
 * A minimal joint-1 sweep used to demonstrate the task scheduling chain
 * end-to-end without needing any hardware besides one ZDT motor on can4.
 * Each ticks_in_state increment is ~200 ms (scheduler period).
 */

#include <stdio.h>
#include <string.h>

#include "app_arm_demo.h"
#include "dev.h"
#include "log.h"

#define DEMO_DEFAULT_JOINT     1
#define DEMO_DEFAULT_SWEEP     30.0f   /* degrees */
#define DEMO_DEFAULT_DWELL     5       /* ticks (~1 s) */

void app_arm_demo_start(ArmDemoTask *task) {
    if (task == NULL) return;
    task->state          = AD_INIT;
    task->ticks_in_state = 0;
    task->joint_index    = DEMO_DEFAULT_JOINT;
    task->sweep_deg      = DEMO_DEFAULT_SWEEP;
    task->dwell_ticks    = DEMO_DEFAULT_DWELL;
    log_info("app.arm_demo",
             "start FSM joint=%d sweep=%.1fdeg dwell=%d ticks",
             task->joint_index, (double)task->sweep_deg, task->dwell_ticks);
}

bool app_arm_demo_done(const ArmDemoTask *task) {
    return task != NULL && task->state == AD_DONE;
}

static void next_state(ArmDemoTask *task, ArmDemoState st) {
    task->state          = st;
    task->ticks_in_state = 0;
}

bool app_arm_demo_step(ArmDemoTask *task,
                       const DeviceRegistry *reg,
                       char *err,
                       int err_len) {
    (void)reg;
    if (task == NULL) {
        snprintf(err, (size_t)err_len, "task null");
        return false;
    }
    task->ticks_in_state++;

    switch (task->state) {
        case AD_INIT:
            log_info("app.arm_demo", "phase 1/3: move joint %d to +%.1fdeg",
                     task->joint_index, (double)task->sweep_deg);
            if (!arm_move_joint_deg(task->joint_index, task->sweep_deg)) {
                snprintf(err, (size_t)err_len,
                         "arm.move_joint(+%.1f) failed - is proc_arm reachable?",
                         (double)task->sweep_deg);
                next_state(task, AD_FAIL);
                return false;
            }
            next_state(task, AD_DWELL_POS);
            return true;

        case AD_DWELL_POS:
            if (task->ticks_in_state >= task->dwell_ticks) {
                next_state(task, AD_MOVE_NEG);
            }
            return true;

        case AD_MOVE_NEG:
            log_info("app.arm_demo", "phase 2/3: move joint %d to -%.1fdeg",
                     task->joint_index, (double)task->sweep_deg);
            if (!arm_move_joint_deg(task->joint_index, -task->sweep_deg)) {
                snprintf(err, (size_t)err_len,
                         "arm.move_joint(-%.1f) failed",
                         (double)task->sweep_deg);
                next_state(task, AD_FAIL);
                return false;
            }
            next_state(task, AD_DWELL_NEG);
            return true;

        case AD_DWELL_NEG:
            if (task->ticks_in_state >= task->dwell_ticks) {
                next_state(task, AD_MOVE_ZERO);
            }
            return true;

        case AD_MOVE_ZERO:
            log_info("app.arm_demo", "phase 3/3: move joint %d back to 0deg",
                     task->joint_index);
            if (!arm_move_joint_deg(task->joint_index, 0.0f)) {
                snprintf(err, (size_t)err_len, "arm.move_joint(0) failed");
                next_state(task, AD_FAIL);
                return false;
            }
            next_state(task, AD_DONE);
            return true;

        /* MOVE_POS is reachable only if a future caller skips AD_INIT.
         * Treat it as a no-op alias for AD_INIT to keep the FSM total. */
        case AD_MOVE_POS:
            next_state(task, AD_INIT);
            return true;

        case AD_DONE:
            return true;

        case AD_FAIL:
        default:
            return false;
    }
}
