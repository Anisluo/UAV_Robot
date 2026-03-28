#include "dev.h"
#include "log.h"

void dev_emergency_stop_all(void) {
    log_error("core.dev.relay", "EMERGENCY STOP: stop arm/car and force safe GPIO state");
}
