#ifndef UAV_MSG_TYPES_H
#define UAV_MSG_TYPES_H

#include <stdint.h>

#define UAV_TEXT_MAX 128

typedef enum {
    CMD_INVALID = 0,
    CMD_START_BATTERY_PICK,
    CMD_EMERGENCY_STOP,
    CMD_RESET_FAULT,
    CMD_SHUTDOWN
} CommandType;

typedef enum {
    TASK_NONE = 0,
    TASK_BATTERY_PICK
} TaskType;

typedef enum {
    EVT_NONE = 0,
    EVT_IO_CMD_RAW,
    EVT_TIMER_TICK,
    EVT_CMD_REQUEST,
    EVT_CMD_ACK,
    EVT_CMD_NACK,
    EVT_TASK_REQUEST,
    EVT_TASK_DONE,
    EVT_TASK_FAIL,
    EVT_EMERGENCY_STOP,
    EVT_RESET_FAULT,
    EVT_SHUTDOWN
} EventType;

typedef struct {
    EventType type;
    uint64_t ts_ms;
    union {
        struct {
            char text[UAV_TEXT_MAX];
        } io;
        struct {
            CommandType cmd;
        } command;
        struct {
            CommandType cmd;
            char reason[UAV_TEXT_MAX];
        } feedback;
        struct {
            TaskType task;
        } task;
        struct {
            char reason[UAV_TEXT_MAX];
        } failure;
    } data;
} Event;

#endif
