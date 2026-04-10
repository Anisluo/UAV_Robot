#include "arm_controller.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>

extern "C" {
#include "dev.h"
#include "log.h"
}

namespace {
constexpr const char *kTag = "proc_arm.controller";
constexpr size_t kJointCount = 6;
constexpr double kMotorPulsesPerRev = 3200.0;
constexpr std::array<double, kJointCount> kGearRatios = {25.0, 20.0, 25.0, 10.0, 4.0, 1.0};
constexpr std::array<double, kJointCount> kJointMinDeg = {-180.0, 0.0, -80.0, -30.0, -110.0, -30.0};
constexpr std::array<double, kJointCount> kJointMaxDeg = {180.0, 180.0, 83.0, 305.0, 110.0, 305.0};
constexpr double kBaseOffsetMm = 55.0;
constexpr double kBaseHeightMm = 166.0;
constexpr double kLink2Mm = 200.0;
constexpr double kLink3AMm = 56.0;
constexpr double kLink4DMm = 192.0;
constexpr double kToolOffsetMm = 55.0;
constexpr double kDefaultJointSpeedDegPerSec = 60.0;
constexpr double kMinEtaSeconds = 0.05;
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, kJointCount> kPostHomeSafeDeg = {180.0, 90.0, 83.0, 30.0, 110.0, 30.0};

struct Pose {
    double x_mm;
    double y_mm;
    double z_mm;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
};

double clampValue(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double getenvDouble(const char *name, double fallback) {
    const char *text = std::getenv(name);
    char *end = nullptr;
    double value;

    if (text == nullptr || text[0] == '\0') {
        return fallback;
    }
    value = std::strtod(text, &end);
    if (end == text || (end != nullptr && *end != '\0')) {
        return fallback;
    }
    return value;
}

std::array<double, 6> getPostHomeSafeDeg() {
    const char *env_names[kJointCount] = {
        "UAV_ARM_J1_POST_HOME_DEG",
        "UAV_ARM_J2_POST_HOME_DEG",
        "UAV_ARM_J3_POST_HOME_DEG",
        "UAV_ARM_J4_POST_HOME_DEG",
        "UAV_ARM_J5_POST_HOME_DEG",
        "UAV_ARM_J6_POST_HOME_DEG",
    };
    std::array<double, 6> targets = kPostHomeSafeDeg;

    for (size_t i = 0; i < kJointCount; ++i) {
        targets[i] = getenvDouble(env_names[i], targets[i]);
    }
    return targets;
}

double estimateMotionTime(const std::array<double, 6> &from_deg,
                          const std::array<double, 6> &to_deg,
                          double speed_ratio) {
    double max_delta = 0.0;
    speed_ratio = clampValue(speed_ratio, 0.01, 1.0);
    for (size_t i = 0; i < kJointCount; ++i) {
        max_delta = std::max(max_delta, std::fabs(to_deg[i] - from_deg[i]));
    }
    return std::max(kMinEtaSeconds, max_delta / (kDefaultJointSpeedDegPerSec * speed_ratio));
}

bool isValidRotationOrder(const std::string &rotation_order) {
    return rotation_order == "zyx" || rotation_order == "xyz";
}

void rotationMatrixToEuler(const std::array<double, 9> &r,
                           const std::string &rotation_order,
                           double *roll_deg,
                           double *pitch_deg,
                           double *yaw_deg) {
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;

    if (rotation_order == "zyx") {
        pitch = std::asin(clampValue(-r[6], -1.0, 1.0));
        yaw = std::atan2(r[3], r[0]);
        roll = std::atan2(r[7], r[8]);
    } else {
        pitch = std::asin(clampValue(r[2], -1.0, 1.0));
        roll = std::atan2(-r[5], r[8]);
        yaw = std::atan2(-r[1], r[0]);
    }

    *roll_deg = roll * 180.0 / kPi;
    *pitch_deg = pitch * 180.0 / kPi;
    *yaw_deg = yaw * 180.0 / kPi;
}

bool solvePositionIk(double x_mm, double y_mm, double z_mm, std::array<double, 6> *out_joints_deg) {
    const double link3_eff_mm = std::sqrt(kLink3AMm * kLink3AMm + kLink4DMm * kLink4DMm);
    const double link3_beta_rad = std::atan2(kLink4DMm, kLink3AMm);
    double radial_mm = std::hypot(x_mm, y_mm);
    double wrist_r_mm = radial_mm - kBaseOffsetMm - kToolOffsetMm;
    double wrist_z_mm = z_mm - kBaseHeightMm;
    double dist_sq;
    double cos_phi;
    double phi_rad;
    double shoulder_rad;

    if (out_joints_deg == nullptr) {
        return false;
    }
    if (wrist_r_mm < 1.0) {
        wrist_r_mm = 1.0;
    }

    dist_sq = wrist_r_mm * wrist_r_mm + wrist_z_mm * wrist_z_mm;
    cos_phi = (dist_sq - kLink2Mm * kLink2Mm - link3_eff_mm * link3_eff_mm) /
              (2.0 * kLink2Mm * link3_eff_mm);
    if (cos_phi < -1.0 || cos_phi > 1.0) {
        return false;
    }

    cos_phi = clampValue(cos_phi, -1.0, 1.0);
    phi_rad = std::atan2(std::sqrt(1.0 - cos_phi * cos_phi), cos_phi);
    shoulder_rad = std::atan2(wrist_z_mm, wrist_r_mm) -
                   std::atan2(link3_eff_mm * std::sin(phi_rad),
                              kLink2Mm + link3_eff_mm * std::cos(phi_rad));

    (*out_joints_deg)[0] = std::atan2(y_mm, x_mm) * 180.0 / kPi;
    (*out_joints_deg)[1] = shoulder_rad * 180.0 / kPi;
    (*out_joints_deg)[2] = (phi_rad - link3_beta_rad) * 180.0 / kPi;
    (*out_joints_deg)[3] = 0.0;
    (*out_joints_deg)[4] = 0.0;
    (*out_joints_deg)[5] = 0.0;
    return true;
}

bool forwardPoseFromJoints(const std::array<double, 6> &joints_deg,
                           const std::string &rotation_order,
                           Pose *pose) {
    std::array<double, 6> q = joints_deg;
    std::array<double, 9> r = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
    const double q1 = q[0] * kPi / 180.0;
    const double q2 = q[1] * kPi / 180.0;
    const double q3 = q[2] * kPi / 180.0;
    const double radial_mm =
        kBaseOffsetMm + kToolOffsetMm + kLink2Mm * std::cos(q2) + kLink3AMm * std::cos(q2 + q3);
    const double z_mm = kBaseHeightMm + kLink2Mm * std::sin(q2) + kLink3AMm * std::sin(q2 + q3) +
                        kLink4DMm;

    if (pose == nullptr || !isValidRotationOrder(rotation_order)) {
        return false;
    }

    pose->x_mm = radial_mm * std::cos(q1);
    pose->y_mm = radial_mm * std::sin(q1);
    pose->z_mm = z_mm;

    {
        const double c4 = std::cos(q[3] * kPi / 180.0);
        const double s4 = std::sin(q[3] * kPi / 180.0);
        const double c5 = std::cos(q[4] * kPi / 180.0);
        const double s5 = std::sin(q[4] * kPi / 180.0);
        const double c6 = std::cos(q[5] * kPi / 180.0);
        const double s6 = std::sin(q[5] * kPi / 180.0);

        r = {
            c4 * c5, c4 * s5 * s6 - s4 * c6, c4 * s5 * c6 + s4 * s6,
            s4 * c5, s4 * s5 * s6 + c4 * c6, s4 * s5 * c6 - c4 * s6,
            -s5,     c5 * s6,                c5 * c6
        };
    }
    rotationMatrixToEuler(r, rotation_order, &pose->roll_deg, &pose->pitch_deg, &pose->yaw_deg);
    return true;
}

std::array<double, 16> transformFromPose(const Pose &pose) {
    const double roll = pose.roll_deg * kPi / 180.0;
    const double pitch = pose.pitch_deg * kPi / 180.0;
    const double yaw = pose.yaw_deg * kPi / 180.0;
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    return {
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, pose.x_mm / 1000.0,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, pose.y_mm / 1000.0,
        -sp,     cp * sr,                cp * cr,                pose.z_mm / 1000.0,
        0.0,     0.0,                    0.0,                    1.0
    };
}
}

ArmController &ArmController::instance() {
    static ArmController controller;
    return controller;
}

ArmController::ArmController()
    : current_joints_deg_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      zero_offsets_deg_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      estop_enabled_(false),
      free_mode_enabled_(false),
      servo_angle_deg_(0) {}

bool ArmController::home() {
    const std::array<double, 6> safe_deg = getPostHomeSafeDeg();

    for (size_t i = 0; i < kJointCount; ++i) {
        const int joint_index = static_cast<int>(i + 1U);
        if (!homeJoint(joint_index)) {
            log_error(kTag, "home failed at joint %d", joint_index);
            return false;
        }
        if (!moveJointDeg(joint_index, safe_deg[i])) {
            log_error(kTag,
                      "post-home safe move failed at joint %d target=%.2f",
                      joint_index,
                      safe_deg[i]);
            return false;
        }
    }

    current_joints_deg_ = safe_deg;
    return true;
}

bool ArmController::homeJoint(int joint_index) {
    if (!arm_home_joint(joint_index)) {
        return false;
    }
    if (joint_index >= 1 && joint_index <= static_cast<int>(kJointCount)) {
        current_joints_deg_[static_cast<size_t>(joint_index - 1)] = 0.0;
        zero_offsets_deg_[static_cast<size_t>(joint_index - 1)] = 0.0;
    }
    return true;
}

bool ArmController::stop() {
    return arm_stop();
}

bool ArmController::emergencyStop(bool enable) {
    estop_enabled_ = enable;
    if (enable) {
        dev_emergency_stop_all();
        return arm_stop();
    }
    return true;
}

bool ArmController::setFreeMode(bool enable) {
    free_mode_enabled_ = enable;
    log_info(kTag, "set free mode=%s", enable ? "true" : "false");
    return true;
}

bool ArmController::movePose(const std::string &pose_name) {
    if (!arm_move(pose_name.c_str())) {
        return false;
    }
    if (pose_name == "zero" || pose_name == "home") {
        current_joints_deg_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    } else if (pose_name == "approach") {
        current_joints_deg_ = {0.0, 55.0, -30.0, 0.0, 0.0, 0.0};
    } else if (pose_name == "lift") {
        current_joints_deg_ = {0.0, 35.0, -20.0, 0.0, 0.0, 0.0};
    }
    return true;
}

bool ArmController::moveJointDeg(int joint_index, double target_deg) {
    double physical_target_deg = target_deg;

    if (estop_enabled_ || free_mode_enabled_) {
        return false;
    }
    if (joint_index >= 1 && joint_index <= static_cast<int>(kJointCount)) {
        physical_target_deg += zero_offsets_deg_[static_cast<size_t>(joint_index - 1)];
    }
    if (!arm_move_joint_deg(joint_index, static_cast<float>(physical_target_deg))) {
        return false;
    }
    if (joint_index >= 1 && joint_index <= static_cast<int>(kJointCount)) {
        current_joints_deg_[static_cast<size_t>(joint_index - 1)] = physical_target_deg;
    }
    return true;
}

bool ArmController::moveJointsInternal(const std::array<double, 6> &joints_deg,
                                       double speed_ratio,
                                       bool update_cache,
                                       double *eta_s) {
    float targets[6];

    if (estop_enabled_ || free_mode_enabled_) {
        return false;
    }
    for (size_t i = 0; i < kJointCount; ++i) {
        const double physical_target_deg = joints_deg[i] + zero_offsets_deg_[i];
        if (physical_target_deg < kJointMinDeg[i] || physical_target_deg > kJointMaxDeg[i]) {
            return false;
        }
        targets[i] = static_cast<float>(physical_target_deg);
    }
    if (eta_s != nullptr) {
        std::array<double, 6> physical_targets = joints_deg;
        for (size_t i = 0; i < kJointCount; ++i) {
            physical_targets[i] += zero_offsets_deg_[i];
        }
        *eta_s = estimateMotionTime(current_joints_deg_, physical_targets, speed_ratio);
    }
    if (!arm_move_joints_deg(targets)) {
        return false;
    }
    if (update_cache) {
        for (size_t i = 0; i < kJointCount; ++i) {
            current_joints_deg_[i] = static_cast<double>(targets[i]);
        }
    }
    return true;
}

bool ArmController::moveJointsDeg(const std::array<double, 6> &joints_deg, double speed_ratio, double *eta_s) {
    return moveJointsInternal(joints_deg, speed_ratio, true, eta_s);
}

bool ArmController::moveToXyz(double x_mm, double y_mm, double z_mm, double *eta_s) {
    std::array<double, 6> joints_deg;

    if (!solvePositionIk(x_mm, y_mm, z_mm, &joints_deg)) {
        return false;
    }
    if (eta_s != nullptr) {
        *eta_s = estimateMotionTime(current_joints_deg_, joints_deg, 1.0);
    }
    if (!arm_move_to_xyz(static_cast<float>(x_mm), static_cast<float>(y_mm), static_cast<float>(z_mm))) {
        return false;
    }
    current_joints_deg_[0] = joints_deg[0];
    current_joints_deg_[1] = joints_deg[1];
    current_joints_deg_[2] = joints_deg[2];
    current_joints_deg_[3] = 0.0;
    current_joints_deg_[4] = 0.0;
    current_joints_deg_[5] = 0.0;
    return true;
}

bool ArmController::moveToPose6d(double x_mm,
                                 double y_mm,
                                 double z_mm,
                                 double roll_deg,
                                 double pitch_deg,
                                 double yaw_deg,
                                 const std::string &rotation_order,
                                 double speed_ratio,
                                 double *eta_s) {
    std::array<double, 6> joints_deg;

    if (!isValidRotationOrder(rotation_order) ||
        !solvePositionIk(x_mm, y_mm, z_mm, &joints_deg)) {
        return false;
    }
    joints_deg[3] = clampValue(roll_deg, kJointMinDeg[3], kJointMaxDeg[3]);
    joints_deg[4] = clampValue(pitch_deg, kJointMinDeg[4], kJointMaxDeg[4]);
    joints_deg[5] = clampValue(yaw_deg, kJointMinDeg[5], kJointMaxDeg[5]);

    if (eta_s != nullptr) {
        *eta_s = estimateMotionTime(current_joints_deg_, joints_deg, speed_ratio);
    }
    if (!arm_move_to_pose6d(static_cast<float>(x_mm),
                            static_cast<float>(y_mm),
                            static_cast<float>(z_mm),
                            static_cast<float>(joints_deg[3]),
                            static_cast<float>(joints_deg[4]),
                            static_cast<float>(joints_deg[5]))) {
        return false;
    }
    current_joints_deg_ = joints_deg;
    return true;
}

bool ArmController::moveLinearPose6d(double x_mm,
                                     double y_mm,
                                     double z_mm,
                                     double roll_deg,
                                     double pitch_deg,
                                     double yaw_deg,
                                     const std::string &rotation_order,
                                     double *eta_s) {
    return moveToPose6d(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, rotation_order, 1.0, eta_s);
}

bool ArmController::dynamicMove(const std::array<double, 6> &goal_pos,
                                const std::array<double, 6> &current_pulses,
                                const std::array<double, 6> &max_speed,
                                double *eta_s) {
    std::array<double, 6> seed = current_joints_deg_;

    for (size_t i = 0; i < kJointCount; ++i) {
        const double pulses_per_degree = (kMotorPulsesPerRev * kGearRatios[i]) / 360.0;
        if (std::fabs(current_pulses[i]) > 1.0) {
            seed[i] = current_pulses[i] / pulses_per_degree;
        }
        if (std::fabs(max_speed[i]) > 1e-6) {
            (void)max_speed[i];
        }
    }
    current_joints_deg_ = seed;
    return moveJointsInternal(goal_pos, 1.0, true, eta_s);
}

bool ArmController::setGripperOpen(bool open) {
    return gripper_open(open);
}

bool ArmController::setServoGripper(int angle_deg, double *eta_s) {
    servo_angle_deg_ = static_cast<int>(clampValue(static_cast<double>(angle_deg), 0.0, 110.0));
    if (eta_s != nullptr) {
        *eta_s = 1.0;
    }
    log_info(kTag, "servo gripper target=%ddeg (not yet wired to hardware)", servo_angle_deg_);
    return true;
}

bool ArmController::setCurrentZero(int joint_index) {
    if (joint_index < 1 || joint_index > static_cast<int>(kJointCount)) {
        return false;
    }
    zero_offsets_deg_[static_cast<size_t>(joint_index - 1)] = current_joints_deg_[static_cast<size_t>(joint_index - 1)];
    log_info(kTag,
             "set current zero joint=%d physical=%.2f reported->0",
             joint_index,
             current_joints_deg_[static_cast<size_t>(joint_index - 1)]);
    return true;
}

bool ArmController::resetCurrentZero(int joint_index) {
    if (joint_index < 1 || joint_index > static_cast<int>(kJointCount)) {
        return false;
    }
    zero_offsets_deg_[static_cast<size_t>(joint_index - 1)] = 0.0;
    log_info(kTag, "reset current zero joint=%d", joint_index);
    return true;
}

std::array<double, 6> ArmController::reportedJointsDeg() const {
    std::array<double, 6> reported = current_joints_deg_;
    for (size_t i = 0; i < kJointCount; ++i) {
        reported[i] -= zero_offsets_deg_[i];
    }
    return reported;
}

std::array<double, 6> ArmController::getMotorAngles() const {
    return reportedJointsDeg();
}

bool ArmController::getPose(const std::string &rotation_order, std::array<double, 6> *pose) const {
    Pose local_pose;

    if (pose == nullptr || !forwardPoseFromJoints(current_joints_deg_, rotation_order, &local_pose)) {
        return false;
    }
    *pose = {local_pose.x_mm,
             local_pose.y_mm,
             local_pose.z_mm,
             local_pose.roll_deg,
             local_pose.pitch_deg,
             local_pose.yaw_deg};
    return true;
}

bool ArmController::getTransform(std::array<double, 16> *transform) const {
    Pose local_pose;

    if (transform == nullptr || !forwardPoseFromJoints(current_joints_deg_, "zyx", &local_pose)) {
        return false;
    }
    *transform = transformFromPose(local_pose);
    return true;
}
