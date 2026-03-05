#include "dev.h"
#include "log.h"

bool car_set_velocity(int linear_mm_s, int angular_mdeg_s) {
    log_info("core.dev.car", "car_set_velocity(linear=%d, angular=%d)", linear_mm_s, angular_mdeg_s);
    return true;
}
