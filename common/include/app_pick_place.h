#ifndef UAV_APP_PICK_PLACE_H
#define UAV_APP_PICK_PLACE_H

#include <stdbool.h>
#include "device_registry.h"

/* Vision-guided pick-and-place task.
 *
 * Pipeline:
 *   uav_robotd task FSM  ── (RPC) ──►  proc_grasp
 *   proc_grasp           ── (RPC) ──►  proc_arm   (motion)
 *   proc_grasp           ── (UDP) ◄──  proc_npu   (detections)
 *   proc_npu             ── (SHM) ◄──  proc_realsense (RGB+depth)
 *
 * The uav_robotd task is a thin orchestrator: it kicks off proc_grasp
 * and polls grasp.get_status every tick until state becomes "done" or
 * "fail". All heavy lifting (detection → hand-eye → approach → grasp
 * → place) lives in proc_grasp's own FSM.
 *
 * Trigger command from HostGUI:  task.start(task="pick_place")
 */

typedef enum {
    PP_INIT = 0,
    PP_TRIGGER_GRASP,   /* call grasp.start */
    PP_WAIT_DONE,       /* poll grasp.get_status */
    PP_DONE,
    PP_FAIL
} PickPlaceState;

typedef struct {
    PickPlaceState state;
    int            ticks_in_state;
    int            poll_timeout_ticks;   /* default: 300 ticks (~60 s) */
    char           last_error[128];
} PickPlaceTask;

void app_pick_place_start(PickPlaceTask *task);
bool app_pick_place_step(PickPlaceTask *task,
                         const DeviceRegistry *reg,
                         char *err,
                         int err_len);
bool app_pick_place_done(const PickPlaceTask *task);

#endif
