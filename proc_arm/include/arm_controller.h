#ifndef UAV_PROC_ARM_CONTROLLER_H
#define UAV_PROC_ARM_CONTROLLER_H

#include <array>
#include <string>

class ArmController {
public:
    static ArmController &instance();

    bool home();
    bool homeJoint(int joint_index);
    bool stop();
    bool emergencyStop(bool enable);
    bool setFreeMode(bool enable);

    bool movePose(const std::string &pose_name);
    bool moveJointDeg(int joint_index, double target_deg);
    bool moveJointsDeg(const std::array<double, 6> &joints_deg, double speed_ratio, double *eta_s);
    bool moveToXyz(double x_mm, double y_mm, double z_mm, double *eta_s);
    bool moveToPose6d(double x_mm,
                      double y_mm,
                      double z_mm,
                      double roll_deg,
                      double pitch_deg,
                      double yaw_deg,
                      const std::string &rotation_order,
                      double speed_ratio,
                      double *eta_s);
    bool moveLinearPose6d(double x_mm,
                          double y_mm,
                          double z_mm,
                          double roll_deg,
                          double pitch_deg,
                          double yaw_deg,
                          const std::string &rotation_order,
                          double *eta_s);
    bool dynamicMove(const std::array<double, 6> &goal_pos,
                     const std::array<double, 6> &current_pulses,
                     const std::array<double, 6> &max_speed,
                     double *eta_s);

    bool setGripperOpen(bool open);
    bool setServoGripper(int angle_deg, double *eta_s);

    std::array<double, 6> getMotorAngles() const;
    bool getPose(const std::string &rotation_order, std::array<double, 6> *pose) const;
    bool getTransform(std::array<double, 16> *transform) const;

private:
    ArmController();
    ArmController(const ArmController &) = delete;
    ArmController &operator=(const ArmController &) = delete;

    bool moveJointsInternal(const std::array<double, 6> &joints_deg,
                            double speed_ratio,
                            bool update_cache,
                            double *eta_s);

    std::array<double, 6> current_joints_deg_;
    bool estop_enabled_;
    bool free_mode_enabled_;
    int servo_angle_deg_;
};

#endif
