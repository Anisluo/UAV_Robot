#ifndef UAV_PROC_GRASP_ARM_CLIENT_H
#define UAV_PROC_GRASP_ARM_CLIENT_H

#include <string>

// Thin JSON-RPC client for proc_arm.
//
// Matches the shape of proc_gateway's proc_arm_call / proc_arm_call_result
// helpers so both processes talk to the arm the same way.
class ArmClient {
public:
    // `sock_path` defaults to /tmp/uav_proc_arm.sock (override with env
    // UAV_PROC_ARM_SOCK, same convention as proc_gateway).
    explicit ArmClient(std::string sock_path = "");

    // Fire-and-forget style: returns true iff proc_arm answers ok:true.
    // Each call opens its own socket so we never hold the bus open.
    bool call_ok(const char *method, const char *params_json, int timeout_sec = 30);

    // Helpers wrapping the common methods used by the grasp pipeline.
    bool move_pose6d(double x_mm, double y_mm, double z_mm,
                     double roll_deg, double pitch_deg, double yaw_deg,
                     double speed_ratio = 1.0);
    bool move_xyz(double x_mm, double y_mm, double z_mm);
    bool home();
    bool stop();
    bool emergency_stop();
    bool servo_gripper(int angle_deg);

private:
    std::string sock_path_;
};

#endif
