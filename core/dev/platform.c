#include "dev.h"
#include "log.h"

bool platform_lock(bool lock) {
    log_info("core.dev.platform", "platform_lock(%s)", lock ? "true" : "false");
    return true;
}
