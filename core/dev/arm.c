#include "dev.h"
#include "log.h"

bool arm_move(const char *pose) {
    log_info("core.dev.arm", "arm_move(%s)", pose);
    return true;
}
