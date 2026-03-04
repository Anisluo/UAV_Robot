#ifndef UAV_DEVICE_REGISTRY_H
#define UAV_DEVICE_REGISTRY_H

#include <stdbool.h>

typedef struct {
    bool arm_online;
    bool platform_online;
    bool car_online;
    bool gripper_online;
} DeviceRegistry;

void device_registry_init(DeviceRegistry *reg);
bool device_registry_check_ready(const DeviceRegistry *reg);

#endif
