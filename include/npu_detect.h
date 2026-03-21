#ifndef UAV_NPU_DETECT_H
#define UAV_NPU_DETECT_H

#include <stdbool.h>

/*
 * Read the latest detection result pushed by proc_npu.
 * Binds (or reuses) the app-side datagram socket at UAV_NPU_RESULT_APP_PATH
 * and does a non-blocking recv().
 *
 * Returns true and fills *x_mm/*y_mm/*z_mm if a detection with has_xyz is found.
 * Returns false if no result is available yet.
 *
 * Call once per scheduler tick; not thread-safe (single-threaded app).
 */
bool npu_read_latest_battery(float *x_mm, float *y_mm, float *z_mm);

/* Release the socket (call on shutdown). */
void npu_detect_close(void);

#endif /* UAV_NPU_DETECT_H */
