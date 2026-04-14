#ifndef UAV_PROC_GRASP_ARM_CLIENT_H
#define UAV_PROC_GRASP_ARM_CLIENT_H

#include <array>
#include <string>

// Thin JSON-RPC client for proc_arm.
//
// Matches the shape of proc_gateway's proc_arm_call / proc_arm_call_result
// helpers so both processes talk to the arm the same way.
//
// Motion helpers (move_pose6d, move_xyz, servo_gripper) are *blocking*:
// they parse the eta_s field from proc_arm's response and poll
// arm.get_pose until the arm reaches the target or eta_s elapses.
class ArmClient {
public:
    // `sock_path` defaults to /tmp/uav_proc_arm.sock (override with env
    // UAV_PROC_ARM_SOCK, same convention as proc_gateway).
    explicit ArmClient(std::string sock_path = "");

    // Fire-and-forget style: returns true iff proc_arm answers ok:true.
    // Each call opens its own socket so we never hold the bus open.
    bool call_ok(const char *method, const char *params_json, int timeout_sec = 30);

    // Send a motion RPC that returns {"ok":true,"eta_s":N}. Parses eta_s
    // into *out_eta_s (seconds). Returns true iff ok:true.
    bool call_move(const char *method, const char *params_json,
                   double *out_eta_s, int timeout_sec = 30);

    // Block until the arm is within `tol_mm` of the 6-element target pose,
    // or until `timeout_s` seconds elapse. Returns true on convergence.
    bool wait_pose(const double target[6], double tol_mm, double timeout_s);

    // Read current end-effector pose [x,y,z,roll,pitch,yaw] from proc_arm.
    bool get_pose(std::array<double, 6> &out);

    // Helpers wrapping the common methods used by the grasp pipeline.
    // These block until the motion completes (or fails/times out).
    bool move_pose6d(double x_mm, double y_mm, double z_mm,
                     double roll_deg, double pitch_deg, double yaw_deg,
                     double speed_ratio = 1.0);
    bool move_xyz(double x_mm, double y_mm, double z_mm);
    bool home();
    bool stop();
    bool emergency_stop();
    bool servo_gripper(int angle_deg);

private:
    // Raw RPC: send request, return full response string. Empty on failure.
    std::string raw_request(const char *method, const char *params_json, int timeout_sec = 30);

    std::string sock_path_;
};

#endif
