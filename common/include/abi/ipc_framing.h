#ifndef UAV_ABI_IPC_FRAMING_H
#define UAV_ABI_IPC_FRAMING_H

#include <stdint.h>
#include "msg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UAV_IPC_MAGIC 0x55495043U

/* ── SHM 共享内存 ─────────────────────────────────────────────────── */
#define UAV_SHM_RING_NAME "/uav_rs_ring"

/* ── proc_realsense 控制通道（JSON-RPC, SOCK_STREAM） ──────────────── */
#define UAV_CTRL_PATH_B "/tmp/uav_proc_realsense.ctrl.sock"

/* ── proc_npu 控制通道（JSON-RPC, SOCK_STREAM） ────────────────────── */
#define UAV_CTRL_PATH_C "/tmp/uav_proc_npu.ctrl.sock"

/* ── proc_realsense → proc_gateway 帧就绪通知（eventfd 文件路径） ─── *
 * proc_realsense 创建此文件并将 eventfd 编号写入；                    *
 * proc_gateway 打开后通过 eventfd_read() 监听新帧到达。               *
 * 实际上双方共享同一个 eventfd fd，通过 /proc/PID/fd 传递；           *
 * 简化方案：proc_realsense 将 eventfd 绑定为 Unix socketpair 的替代，  *
 * 此处定义路径供 proc_gateway 打开该 eventfd 对应的匿名文件。          *
 * ------------------------------------------------------------------  *
 * 实现：proc_realsense 写帧后向此路径的 Unix SOCK_DGRAM 发送 8 字节   *
 * 计数（与 eventfd write 语义相同），proc_gateway epoll 监听该 fd。    */
#define UAV_RS_FRAME_NOTIFY_PATH "/tmp/uav_rs_frame_notify.sock"

/* ── proc_npu 推结果路径（SOCK_DGRAM，保持不变） ──────────────────── */
#define UAV_NPU_RESULT_GW_PATH  "/tmp/uav_gw_npu_rx.sock"   /* proc_gateway */
#define UAV_NPU_RESULT_APP_PATH "/tmp/uav_app_npu_rx.sock"  /* uav_robotd   */

/* ── uav_robotd 文本指令端口 ──────────────────────────────────────── */
#define UAV_APP_CMD_PORT 9001U

/* ── 遗留二进制帧头（仅 UavIpcHeader 还在 proc_gateway 旧代码中使用） */
typedef enum {
    UAV_IPC_MSG_NONE = 0,
    UAV_IPC_MSG_CTRL = 1,
    UAV_IPC_MSG_FRAME_READY = 2,
    UAV_IPC_MSG_RESULT = 3
} UavIpcType;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t payload_size;
} UavIpcHeader;

/* UavCtrlPayload 保留，供旧代码编译通过；新 ctrl 通道已改为 JSON-RPC */
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
