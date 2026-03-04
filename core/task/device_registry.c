#include "device_registry.h"

void device_registry_init(DeviceRegistry *reg) {
    reg->arm_online = true;
    reg->platform_online = true;
    reg->car_online = true;
    reg->gripper_online = true;
}

bool device_registry_check_ready(const DeviceRegistry *reg) {
    return reg->arm_online && reg->platform_online && reg->car_online && reg->gripper_online;
}
