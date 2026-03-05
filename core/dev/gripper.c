#include "dev.h"
#include "log.h"

bool gripper_open(bool open) {
    log_info("core.dev.gripper", "gripper_open(%s)", open ? "true" : "false");
    return true;
}
