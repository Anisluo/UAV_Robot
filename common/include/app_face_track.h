#ifndef UAV_APP_FACE_TRACK_H
#define UAV_APP_FACE_TRACK_H

#include <stdbool.h>
#include "device_registry.h"

/* Face-tracking task:
 *
 *   proc_realsense → shm → face_tracker.py → UavCResult (class=100)
 *                                          ↓ Unix dgram
 *                                      uav_robotd task
 *                                          ↓ arm_move_joint_deg
 *                                      proc_arm (J1 base rotation)
 *
 * Each tick (~200 ms) the FSM reads the latest face detection and nudges
 * joint 1 so the face stays centered horizontally in the image. Vertical
 * tracking (J2 or J3) can be added later.
 *
 * The task runs indefinitely. Stopped via ESTOP or when HostGUI switches
 * the NPU strategy away from "face".
 */

typedef enum {
    FT_INIT = 0,
    FT_TRACKING,
    FT_IDLE,          /* no face seen for a while — hold position */
    FT_DONE           /* reserved for explicit stop */
} FaceTrackState;

typedef struct {
    FaceTrackState state;
    int            ticks_in_state;
    int            ticks_since_seen;
    float          current_j1_deg;     /* cached J1 angle */
    int            joint_index;        /* default 1 */
    float          fov_h_deg;          /* camera horizontal FOV (~70 for D435) */
    float          deadzone_frac;      /* image-offset tolerance (fraction) */
    float          max_step_deg;       /* per-tick J1 angular move */
} FaceTrackTask;

void app_face_track_start(FaceTrackTask *task);
bool app_face_track_step(FaceTrackTask *task,
                         const DeviceRegistry *reg,
                         char *err,
                         int err_len);
bool app_face_track_done(const FaceTrackTask *task);

#endif
