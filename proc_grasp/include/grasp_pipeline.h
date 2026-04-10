#ifndef UAV_PROC_GRASP_PIPELINE_H
#define UAV_PROC_GRASP_PIPELINE_H

#include <mutex>
#include <string>

#include "abi/msg_types.h"
#include "arm_client.h"
#include "hand_eye.h"
#include "result_listener.h"

// Grasp planner / executor.
//
// Works in ticks driven by the main loop. Non-blocking: each tick
// inspects the latest detection, advances the FSM if active, and fires
// at most one arm RPC. The heavy (blocking) arm.move_pose6d call is
// issued from the tick thread — OK because arm moves are a sync action
// model anyway and proc_grasp has no other real-time obligations.
//
// State flow (compact version of the full 3D/6D FSM; retries/safeguards
// can be layered on once we have real hardware footage):
//
//   IDLE → CONFIRM → PRE_APPROACH → GRIPPER_OPEN → DESCEND
//        → GRASP → LIFT → RETREAT → DONE
//
// Any failure transitions to FAIL with a human-readable reason stored
// for get_status.
class GraspPipeline {
public:
    enum class State {
        IDLE = 0,
        CONFIRM,
        PRE_APPROACH,
        GRIPPER_OPEN,
        DESCEND,
        GRASP,
        LIFT,
        RETREAT,
        DONE,
        FAIL
    };

    GraspPipeline(ResultListener *listener, ArmClient *arm);

    // Called ~10 Hz from the main loop.
    void tick();

    // Kick off a grasp attempt. `mode` selects 3D (planar) vs 6D (full
    // orientation). Transitions IDLE/DONE/FAIL → CONFIRM.
    bool trigger(UavGraspMode mode, std::string *err);

    // Abort any in-flight attempt. Does NOT stop the arm itself — the
    // caller should send arm.stop separately if needed.
    void abort(const char *reason);

    // Continuous mode: keep re-triggering after each DONE.
    void set_auto(bool on) { std::lock_guard<std::mutex> lk(mu_); auto_mode_ = on; }
    bool auto_mode() const { std::lock_guard<std::mutex> lk(mu_); return auto_mode_; }

    // Hand-eye setter (thread-safe).
    bool set_hand_eye(const std::array<float, 12> &m);

    // Snapshot current status as a JSON object body (no outer braces
    // trimming needed — caller embeds directly).
    std::string status_json() const;

    // Current state for log messages.
    static const char *state_name(State s);

private:
    bool pick_best(const UavCResult &r, UavDetection *out) const;
    void enter(State s);
    void fail(const char *reason);

    ResultListener *listener_;
    ArmClient *arm_;

    mutable std::mutex mu_;
    State state_ = State::IDLE;
    UavGraspMode mode_ = UAV_GRASP_MODE_3D;
    bool auto_mode_ = false;
    std::string last_error_;
    int confirm_count_ = 0;
    // Averaging accumulator (camera frame, mm / deg).
    float cs_x_ = 0.0F, cs_y_ = 0.0F, cs_z_ = 0.0F;
    float cs_roll_ = 0.0F, cs_pitch_ = 0.0F, cs_yaw_ = 0.0F;
    // Committed target in base frame.
    float tgt_x_ = 0.0F, tgt_y_ = 0.0F, tgt_z_ = 0.0F;
    float tgt_roll_ = 0.0F, tgt_pitch_ = 0.0F, tgt_yaw_ = 0.0F;
    HandEye hand_eye_;

    // Counters / telemetry
    uint64_t attempts_ = 0;
    uint64_t successes_ = 0;
    uint64_t failures_ = 0;
};

#endif
