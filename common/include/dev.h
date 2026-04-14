#ifndef UAV_DEV_H
#define UAV_DEV_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool arm_move(const char *pose);
bool arm_move_to_xyz(float x_mm, float y_mm, float z_mm);
bool arm_move_to_pose6d(float x_mm,
                        float y_mm,
                        float z_mm,
                        float roll_deg,
                        float pitch_deg,
                        float yaw_deg);
bool arm_move_joint_deg(int joint_index, float target_deg);
bool arm_move_joints_deg(const float joints_deg[6]);
bool arm_home(void);
bool arm_home_joint(int joint_index);
bool arm_set_zero_params(int joint_index);
bool arm_stop_joint(int joint_index);
bool arm_stop(void);
void arm_request_estop(void);
void arm_clear_estop(void);
bool arm_estop_requested(void);
void arm_set_move_rpm(int rpm);
void arm_set_zero_rpm(int rpm);
int  arm_get_move_rpm(void);
int  arm_get_zero_rpm(void);
/* Read real encoder positions from motors via CAN (ZDT cmd 0x33).
 * Fills angles[6] in degrees.  Returns number of joints that responded. */
int  arm_read_motor_angles(double angles[6]);
bool platform_lock(bool lock);
bool car_set_velocity(int linear_mm_s, int angular_mdeg_s);
bool gripper_open(bool open);
void dev_emergency_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif
