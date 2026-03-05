#ifndef UAV_ABI_IPC_FRAMING_H
#define UAV_ABI_IPC_FRAMING_H

#include <stdint.h>
#include "msg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UAV_IPC_MAGIC 0x55495043U
#define UAV_CTRL_PATH_B "/tmp/uav_proc_realsense.ctrl.sock"
#define UAV_CTRL_PATH_C "/tmp/uav_proc_npu.ctrl.sock"
#define UAV_DATA_NOTIFY_PATH_C "/tmp/uav_proc_npu.data.sock"
#define UAV_SHM_RING_NAME "/uav_rs_ring"

typedef enum {
    UAV_IPC_MSG_NONE = 0,
    UAV_IPC_MSG_CTRL = 1,
    UAV_IPC_MSG_FRAME_READY = 2
} UavIpcType;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t payload_size;
} UavIpcHeader;

typedef struct {
    uint16_t cmd;
    uint16_t target;
    int32_t i32_arg0;
    int32_t i32_arg1;
    float f32_arg0;
    float f32_arg1;
} UavCtrlPayload;

typedef struct {
    uint32_t slot_id;
    uint64_t frame_id;
    uint64_t timestamp_ns;
} UavFrameReadyPayload;

#ifdef __cplusplus
}
#endif

#endif
