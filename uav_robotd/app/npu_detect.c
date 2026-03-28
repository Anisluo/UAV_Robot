#include "npu_detect.h"
#include "abi/ipc_framing.h"
#include "abi/msg_types.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
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

void npu_detect_close(void) {
    if (g_fd >= 0) {
        close(g_fd);
        unlink(UAV_NPU_RESULT_APP_PATH);
        g_fd = -1;
    }
}
