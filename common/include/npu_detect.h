#ifndef UAV_NPU_DETECT_H
#define UAV_NPU_DETECT_H

#include <stdint.h>
#include <stdbool.h>

#include "abi/msg_types.h"

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    bool has_xyz;
    bool has_rpy;
    uint8_t grasp_mode;
} NpuGraspPose;

/*
 * Read the latest detection result pushed by proc_npu.
 * Binds (or reuses) the app-side datagram socket at UAV_NPU_RESULT_APP_PATH
 * and does a non-blocking recv().
 *
 * Returns true and fills x_mm, y_mm, and z_mm when a detection with has_xyz is found.
 * Returns false if no result is available yet.
 *
 * Call once per scheduler tick; not thread-safe (single-threaded app).
 */
bool npu_read_latest_battery(float *x_mm, float *y_mm, float *z_mm);
bool npu_read_latest_grasp_pose(UavGraspMode requested_mode, NpuGraspPose *out_pose);

/*
 * Read latest FACE detection (class_id == 100, published by face_tracker.py).
 * Returns pixel-frame bbox center in *cx_px, *cy_px and the frame size in
 * *frame_w, *frame_h so callers can compute the normalized image offset.
 * Returns false if no face was seen since the last call.
 */
bool npu_read_latest_face(float *cx_px, float *cy_px,
                          float *frame_w, float *frame_h);

/* Release the socket (call on shutdown). */
void npu_detect_close(void);

#endif /* UAV_NPU_DETECT_H */
