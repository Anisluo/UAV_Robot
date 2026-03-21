#ifndef UAV_DEV_H
#define UAV_DEV_H

#include <stdbool.h>

bool arm_move(const char *pose);
bool arm_move_to_xyz(float x_mm, float y_mm, float z_mm);
bool platform_lock(bool lock);
bool car_set_velocity(int linear_mm_s, int angular_mdeg_s);
bool gripper_open(bool open);
void dev_emergency_stop_all(void);

#endif
