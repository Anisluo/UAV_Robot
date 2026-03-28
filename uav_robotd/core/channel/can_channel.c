#include "can_channel.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "can_socketcan.h"
#include "log.h"
#include "proto_platform_lock.h"
#include "proto_zdt_arm.h"

static uint64_t can_channel_now_ms(void) {
    struct timespec ts;

    if (timespec_get(&ts, TIME_UTC) == TIME_UTC) {
        return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
    }
    return 0;
}

static void can_channel_publish_device_feedback(EventBus *bus,
                                                const char *source,
                                                uint8_t addr,
                                                uint8_t cmd,
                                                uint8_t status,
                                                const char *text) {
    Event ev;

    if (bus == NULL || source == NULL || text == NULL) {
        return;
    }

    memset(&ev, 0, sizeof(ev));
    ev.type = EVT_DEVICE_FEEDBACK;
    ev.ts_ms = can_channel_now_ms();
    snprintf(ev.data.device.source, sizeof(ev.data.device.source), "%s", source);
    ev.data.device.addr = addr;
    ev.data.device.cmd = cmd;
    ev.data.device.status = status;
    snprintf(ev.data.device.text, sizeof(ev.data.device.text), "%s", text);
    event_bus_publish(bus, &ev);
}

static const char *zdt_arm_resp_text(ZdtArmResponseType type) {
    switch (type) {
        case ZDT_ARM_RESP_ACK_OK:
            return "ack ok";
        case ZDT_ARM_RESP_ACK_REJECTED:
            return "ack rejected";
        case ZDT_ARM_RESP_ACK_INVALID:
            return "ack invalid";
        case ZDT_ARM_RESP_POSITION_REACHED:
            return "position reached";
        case ZDT_ARM_RESP_UNKNOWN:
        default:
            return "unknown";
    }
}

static const char *platform_lock_resp_text(PlatformLockResponseType type) {
    switch (type) {
        case PLATFORM_LOCK_RESP_ACK_OK:
            return "ack ok";
        case PLATFORM_LOCK_RESP_ACK_REJECTED:
            return "ack rejected";
        case PLATFORM_LOCK_RESP_ACK_INVALID:
            return "ack invalid";
        case PLATFORM_LOCK_RESP_POSITION_REACHED:
            return "position reached";
        case PLATFORM_LOCK_RESP_UNKNOWN:
        default:
            return "unknown";
    }
}

int can_channel_poll(EventBus *bus) {
    CanSocketFrame frame;
    int handled = 0;

    while (can_socketcan_receive(&frame)) {
        if (!frame.is_extended_id) {
            // RoboModule chassis drivers answer with standard CAN frames that we do not
            // currently decode in uav_robotd; ignore them to avoid noisy false-error logs.
            continue;
        }

        if (frame.is_extended_id) {
            ZdtArmResponse arm_resp;
            PlatformLockResponse platform_resp;

            if (proto_zdt_arm_decode(frame.data, frame.len, &arm_resp)) {
                can_channel_publish_device_feedback(bus,
                                                    "zdt_arm",
                                                    arm_resp.addr,
                                                    arm_resp.cmd,
                                                    arm_resp.status,
                                                    zdt_arm_resp_text(arm_resp.type));
                handled++;
                continue;
            }

            if (proto_platform_lock_decode(frame.data, frame.len, &platform_resp)) {
                can_channel_publish_device_feedback(bus,
                                                    "platform_lock",
                                                    platform_resp.addr,
                                                    platform_resp.cmd,
                                                    platform_resp.status,
                                                    platform_lock_resp_text(platform_resp.type));
                handled++;
                continue;
            }
        }

        log_info("core.channel.can",
                 "unhandled can frame id=0x%08X ext=%s len=%u",
                 (unsigned int)frame.can_id,
                 frame.is_extended_id ? "true" : "false",
                 (unsigned int)frame.len);
    }

    return handled;
}
