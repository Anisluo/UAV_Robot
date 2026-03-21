#ifndef UAV_ABI_MSG_TYPES_H
#define UAV_ABI_MSG_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UAV_ABI_VERSION 1U
#define UAV_MAX_DETECTIONS 32U

typedef enum {
    UAV_PROC_UNKNOWN = 0,
    UAV_PROC_A = 1,
    UAV_PROC_B = 2,
    UAV_PROC_C = 3
} UavProcId;

typedef enum {
    UAV_STRATEGY_DEFAULT    = 0,  /* default.rknn – square battery detection */
    UAV_STRATEGY_BATTERY_V2 = 1,  /* battery_v2.rknn */
    UAV_STRATEGY_CUSTOM     = 2   /* custom.rknn */
} UavStrategyId;

typedef enum {
    UAV_CTRL_NONE = 0,
    UAV_CTRL_B_START,
    UAV_CTRL_B_STOP,
    UAV_CTRL_B_SET_PROFILE,
    UAV_CTRL_B_SET_EXPOSURE,
    UAV_CTRL_B_SET_ROI,
    UAV_CTRL_B_SNAPSHOT,
    UAV_CTRL_C_START,
    UAV_CTRL_C_STOP,
    UAV_CTRL_C_LOAD_MODEL,
    UAV_CTRL_C_UNLOAD_MODEL,
    UAV_CTRL_C_SET_THRESHOLD,
    UAV_CTRL_C_SET_RATE,
    UAV_CTRL_C_SET_STRATEGY   /* i32_arg0 = UavStrategyId */
} UavCtrlCmd;

typedef enum {
    UAV_PROC_STATE_INIT = 0,
    UAV_PROC_STATE_IDLE,
    UAV_PROC_STATE_RUNNING,
    UAV_PROC_STATE_ERROR
} UavProcState;

typedef struct {
    uint64_t timestamp_ns;
    float fps;
    uint32_t drop_count;
} UavBHeartbeat;

typedef struct {
    uint64_t timestamp_ns;
    float inference_fps;
    float avg_latency_ms;
} UavCHeartbeat;

typedef struct {
    UavProcState state;
    int32_t error_code;
} UavProcStatus;

typedef struct {
    int32_t class_id;
    float score;
    float x1;
    float y1;
    float x2;
    float y2;
    float x_mm;
    float y_mm;
    float z_mm;
    uint8_t has_xyz;
    uint8_t reserved[3];
} UavDetection;

typedef struct {
    uint64_t frame_id;
    uint32_t num_detections;
    UavDetection detections[UAV_MAX_DETECTIONS];
} UavCResult;

#ifdef __cplusplus
}
#endif

#endif
