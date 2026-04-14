#include "npu_detect.h"
#include "abi/ipc_framing.h"
#include "abi/msg_types.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

static int g_fd = -1;

static bool ensure_socket(void) {
    if (g_fd >= 0) return true;

    g_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (g_fd < 0) return false;

    /* Remove stale socket file before binding */
    unlink(UAV_NPU_RESULT_APP_PATH);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, UAV_NPU_RESULT_APP_PATH,
            sizeof(addr.sun_path) - 1);

    if (bind(g_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(g_fd);
        g_fd = -1;
        return false;
    }
    /* Allow non-root clients (face_tracker.py runs as ubuntu) to sendto. */
    (void)chmod(UAV_NPU_RESULT_APP_PATH, 0666);

    /* Non-blocking */
    int flags = 0;
    /* fcntl not available without <fcntl.h> – use MSG_DONTWAIT in recv instead */
    (void)flags;
    return true;
}

bool npu_read_latest_battery(float *x_mm, float *y_mm, float *z_mm) {
    if (!ensure_socket()) return false;

    UavCResult r;
    ssize_t n;
    bool found = false;
    UavCResult latest;

    /* Drain all pending datagrams, keep only the latest */
    while ((n = recv(g_fd, &r, sizeof(r), MSG_DONTWAIT)) == (ssize_t)sizeof(r)) {
        for (uint32_t i = 0; i < r.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
            if (r.detections[i].has_xyz) {
                latest = r;
                found  = true;
                break;
            }
        }
    }

    if (!found) return false;

    /* Return the 3D pose of the first detection with xyz */
    for (uint32_t i = 0; i < latest.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
        if (latest.detections[i].has_xyz) {
            *x_mm = latest.detections[i].x_mm;
            *y_mm = latest.detections[i].y_mm;
            *z_mm = latest.detections[i].z_mm;
            return true;
        }
    }
    return false;
}

bool npu_read_latest_grasp_pose(UavGraspMode requested_mode, NpuGraspPose *out_pose) {
    UavCResult r;
    ssize_t n;
    bool found = false;
    UavCResult latest;

    if (out_pose == NULL) return false;
    memset(out_pose, 0, sizeof(*out_pose));
    if (!ensure_socket()) return false;

    while ((n = recv(g_fd, &r, sizeof(r), MSG_DONTWAIT)) == (ssize_t)sizeof(r)) {
        for (uint32_t i = 0; i < r.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
            const UavDetection *det = &r.detections[i];
            if (!det->has_xyz) continue;
            if (requested_mode == UAV_GRASP_MODE_6D && !det->has_rpy) continue;
            latest = r;
            found = true;
            break;
        }
    }

    if (!found) return false;

    for (uint32_t i = 0; i < latest.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
        const UavDetection *det = &latest.detections[i];
        if (!det->has_xyz) continue;
        if (requested_mode == UAV_GRASP_MODE_6D && !det->has_rpy) continue;

        out_pose->x_mm = det->x_mm;
        out_pose->y_mm = det->y_mm;
        out_pose->z_mm = det->z_mm;
        out_pose->roll_deg = det->roll_deg;
        out_pose->pitch_deg = det->pitch_deg;
        out_pose->yaw_deg = det->yaw_deg;
        out_pose->has_xyz = det->has_xyz != 0;
        out_pose->has_rpy = det->has_rpy != 0;
        out_pose->grasp_mode = det->grasp_mode;
        return true;
    }
    return false;
}

#define UAV_FACE_CLASS_ID 100

bool npu_read_latest_face(float *cx_px, float *cy_px,
                          float *frame_w, float *frame_h) {
    UavCResult r;
    ssize_t n;
    bool found = false;
    UavCResult latest;

    if (cx_px == NULL || cy_px == NULL) return false;
    if (!ensure_socket()) return false;

    /* Drain datagrams, keep last one that contains a face detection. */
    while ((n = recv(g_fd, &r, sizeof(r), MSG_DONTWAIT)) == (ssize_t)sizeof(r)) {
        for (uint32_t i = 0; i < r.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
            if (r.detections[i].class_id == UAV_FACE_CLASS_ID) {
                latest = r;
                found  = true;
                break;
            }
        }
    }
    if (!found) return false;

    /* Return the first face (largest area would be better but this is
     * simple and adequate for a single-subject tracker). */
    for (uint32_t i = 0; i < latest.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
        const UavDetection *d = &latest.detections[i];
        if (d->class_id != UAV_FACE_CLASS_ID) continue;
        *cx_px = (d->x1 + d->x2) * 0.5F;
        *cy_px = (d->y1 + d->y2) * 0.5F;
        /* x2/y2 gives us an upper bound on frame size when cx/cy are at
         * the center of the image — but we can't know exact frame w/h
         * from the detection alone. Use sensible defaults unless the
         * caller already has them: 640x480 matches RealSense default. */
        if (frame_w) *frame_w = 640.0F;
        if (frame_h) *frame_h = 480.0F;
        return true;
    }
    return false;
}

void npu_detect_close(void) {
    if (g_fd >= 0) {
        close(g_fd);
        unlink(UAV_NPU_RESULT_APP_PATH);
        g_fd = -1;
    }
}
