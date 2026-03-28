#include "dev.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "can_socketcan.h"
#include "log.h"
#include "proto_platform_lock.h"

#define PLATFORM_LOCK_DEFAULT_ADDR 4
#define PLATFORM_LOCK_RPM 200U
#define PLATFORM_LOCK_ACC 20U
#define PLATFORM_LOCK_PULSES_LOCKED 3200U
#define PLATFORM_LOCK_PULSES_UNLOCKED 0U

static int platform_getenv_int(const char *name, int fallback) {
    const char *text = getenv(name);
    char *end = NULL;
    long value;

    if (text == NULL || text[0] == '\0') {
        return fallback;
    }

    value = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return fallback;
    }
    return (int)value;
}

static bool platform_send_batch(const PlatformLockCanBatch *batch) {
    size_t i;

    if (batch == NULL) {
        return false;
    }
    for (i = 0; i < batch->count; ++i) {
        if (!can_socketcan_send(batch->frames[i].can_id,
                                batch->frames[i].is_extended_id,
                                batch->frames[i].data,
                                batch->frames[i].len)) {
            return false;
        }
    }
    return true;
}

bool platform_lock(bool lock) {
    PlatformLockCanBatch batch;
    uint8_t addr = (uint8_t)platform_getenv_int("UAV_PLATFORM_LOCK_ADDR", PLATFORM_LOCK_DEFAULT_ADDR);
    uint32_t target_pulses = (uint32_t)platform_getenv_int(lock ? "UAV_PLATFORM_LOCK_PULSES" : "UAV_PLATFORM_UNLOCK_PULSES",
                                                           lock ? (int)PLATFORM_LOCK_PULSES_LOCKED
                                                                : (int)PLATFORM_LOCK_PULSES_UNLOCKED);

    if (!proto_platform_lock_encode_enable(addr, true, false, &batch)) {
        return false;
    }
    if (!platform_send_batch(&batch)) {
        return false;
    }

    if (!proto_platform_lock_encode_position(addr,
                                             false,
                                             PLATFORM_LOCK_RPM,
                                             PLATFORM_LOCK_ACC,
                                             target_pulses,
                                             true,
                                             false,
                                             &batch)) {
        return false;
    }
    if (!platform_send_batch(&batch)) {
        return false;
    }

    log_info("core.dev.platform",
             "platform_lock(%s) target_pulses=%u",
             lock ? "true" : "false",
             (unsigned int)target_pulses);
    return true;
}
