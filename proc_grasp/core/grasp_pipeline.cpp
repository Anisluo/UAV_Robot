#include "grasp_pipeline.h"

#include <cmath>
#include <cstdio>
#include <cstring>

// Tunable offsets (mirrors app_arm_grasp.h constants so both the inline
// uav_robotd FSM and proc_grasp land at similar grasp poses).
namespace {
constexpr int   kConfirmFrames       = 3;      // frames to average
constexpr float kPreApproachZOffset  = 80.0F;  // mm above target
constexpr float kLiftZ               = 350.0F; // mm, absolute Z after grasp
constexpr float kRetreatZ            = 280.0F; // mm, safe retreat Z

constexpr float kWsZMin              = 30.0F;
constexpr float kWsZMax              = 650.0F;
constexpr float kWsRMin              = 80.0F;
constexpr float kWsRMax              = 440.0F;

constexpr int   kGripperOpenDeg      = 90;
constexpr int   kGripperCloseDeg     = 0;
}  // namespace

GraspPipeline::GraspPipeline(ResultListener *listener, ArmClient *arm)
    : listener_(listener), arm_(arm) {}

const char *GraspPipeline::state_name(State s)
{
    switch (s) {
        case State::IDLE:         return "idle";
        case State::CONFIRM:      return "confirm";
        case State::PRE_APPROACH: return "pre_approach";
        case State::GRIPPER_OPEN: return "gripper_open";
        case State::DESCEND:      return "descend";
        case State::GRASP:        return "grasp";
        case State::LIFT:         return "lift";
        case State::RETREAT:      return "retreat";
        case State::DONE:         return "done";
        case State::FAIL:         return "fail";
    }
    return "unknown";
}

bool GraspPipeline::pick_best(const UavCResult &r, UavDetection *out) const
{
    if (out == nullptr) return false;
    const UavDetection *best = nullptr;
    for (uint32_t i = 0; i < r.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
        const UavDetection &d = r.detections[i];
        if (d.has_xyz == 0U) continue;  // need 3D to grasp
        if (best == nullptr || d.score > best->score) {
            best = &d;
        }
    }
    if (best == nullptr) return false;
    *out = *best;
    return true;
}

bool GraspPipeline::trigger(UavGraspMode mode, std::string *err)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (state_ != State::IDLE && state_ != State::DONE && state_ != State::FAIL) {
        if (err != nullptr) *err = "busy";
        return false;
    }
    mode_ = mode;
    confirm_count_ = 0;
    cs_x_ = cs_y_ = cs_z_ = 0.0F;
    cs_roll_ = cs_pitch_ = cs_yaw_ = 0.0F;
    last_error_.clear();
    state_ = State::CONFIRM;
    ++attempts_;
    std::fprintf(stderr, "proc_grasp: trigger mode=%s\n",
                 mode == UAV_GRASP_MODE_6D ? "6D" : "3D");
    return true;
}

void GraspPipeline::abort(const char *reason)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (state_ == State::IDLE) return;
    state_ = State::IDLE;
    last_error_ = reason != nullptr ? reason : "aborted";
    std::fprintf(stderr, "proc_grasp: abort: %s\n", last_error_.c_str());
}

void GraspPipeline::enter(State s)
{
    {
        std::lock_guard<std::mutex> lk(mu_);
        state_ = s;
    }
    std::fprintf(stderr, "proc_grasp: state -> %s\n", state_name(s));
}

void GraspPipeline::fail(const char *reason)
{
    {
        std::lock_guard<std::mutex> lk(mu_);
        last_error_ = reason != nullptr ? reason : "unknown";
        ++failures_;
        state_ = State::FAIL;
    }
    std::fprintf(stderr, "proc_grasp: FAIL %s\n", reason ? reason : "unknown");
}

bool GraspPipeline::set_hand_eye(const std::array<float, 12> &m)
{
    std::lock_guard<std::mutex> lk(mu_);
    return hand_eye_.set(m);
}

std::string GraspPipeline::status_json() const
{
    std::lock_guard<std::mutex> lk(mu_);
    char buf[512];
    std::snprintf(buf, sizeof(buf),
        "{\"state\":\"%s\",\"mode\":\"%s\",\"auto\":%s,"
        "\"attempts\":%llu,\"successes\":%llu,\"failures\":%llu,"
        "\"confirm\":%d,"
        "\"target\":{\"x_mm\":%.2f,\"y_mm\":%.2f,\"z_mm\":%.2f,"
        "\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f},"
        "\"error\":\"%s\"}",
        state_name(state_),
        mode_ == UAV_GRASP_MODE_6D ? "6D" : "3D",
        auto_mode_ ? "true" : "false",
        static_cast<unsigned long long>(attempts_),
        static_cast<unsigned long long>(successes_),
        static_cast<unsigned long long>(failures_),
        confirm_count_,
        tgt_x_, tgt_y_, tgt_z_,
        tgt_roll_, tgt_pitch_, tgt_yaw_,
        last_error_.c_str());
    return buf;
}

void GraspPipeline::tick()
{
    // Snapshot state under the lock, then release it before doing any
    // blocking arm RPCs. This keeps get_status / trigger responsive.
    State st;
    UavGraspMode mode;
    bool do_auto;
    {
        std::lock_guard<std::mutex> lk(mu_);
        st = state_;
        mode = mode_;
        do_auto = auto_mode_;
    }

    switch (st) {
        case State::IDLE:
            if (do_auto) {
                std::string err;
                trigger(mode, &err);
            }
            return;

        case State::DONE: {
            if (do_auto) {
                // Drop straight into a new attempt.
                std::lock_guard<std::mutex> lk(mu_);
                state_ = State::IDLE;
            }
            return;
        }

        case State::FAIL: {
            if (do_auto) {
                // Cooldown — operator must explicitly retrigger after a
                // failure to avoid runaway behavior.
            }
            return;
        }

        case State::CONFIRM: {
            UavCResult result{};
            uint64_t age = 0;
            if (!listener_->latest(&result, &age)) return;
            if (age > 1500U) {
                // stale: wait for a fresh frame
                return;
            }
            UavDetection best{};
            if (!pick_best(result, &best)) return;

            bool committed = false;
            bool oob = false;
            {
                std::lock_guard<std::mutex> lk(mu_);
                cs_x_     += best.x_mm;
                cs_y_     += best.y_mm;
                cs_z_     += best.z_mm;
                cs_roll_  += best.roll_deg;
                cs_pitch_ += best.pitch_deg;
                cs_yaw_   += best.yaw_deg;
                ++confirm_count_;

                if (confirm_count_ >= kConfirmFrames) {
                    const float k = static_cast<float>(confirm_count_);
                    const float cx = cs_x_ / k;
                    const float cy = cs_y_ / k;
                    const float cz = cs_z_ / k;
                    hand_eye_.transform_point(cx, cy, cz, &tgt_x_, &tgt_y_, &tgt_z_);
                    if (mode == UAV_GRASP_MODE_6D && best.has_rpy != 0U) {
                        tgt_roll_  = cs_roll_  / k;
                        tgt_pitch_ = cs_pitch_ / k;
                        tgt_yaw_   = cs_yaw_   / k;
                    } else {
                        tgt_roll_  = 180.0F;   // fixed top-down grasp
                        tgt_pitch_ = 0.0F;
                        tgt_yaw_   = 0.0F;
                    }
                    const float r_xy = std::sqrt(tgt_x_ * tgt_x_ + tgt_y_ * tgt_y_);
                    oob = (tgt_z_ < kWsZMin || tgt_z_ > kWsZMax ||
                           r_xy  < kWsRMin || r_xy  > kWsRMax);
                    committed = !oob;
                }
            }
            if (oob) {
                fail("target out of workspace");
            } else if (committed) {
                enter(State::PRE_APPROACH);
            }
            return;
        }

        case State::PRE_APPROACH: {
            float x, y, z, r, p, yw;
            {
                std::lock_guard<std::mutex> lk(mu_);
                x = tgt_x_;
                y = tgt_y_;
                z = tgt_z_ + kPreApproachZOffset;
                r = tgt_roll_;
                p = tgt_pitch_;
                yw = tgt_yaw_;
            }
            if (!arm_->move_pose6d(x, y, z, r, p, yw)) {
                fail("pre_approach move failed");
                return;
            }
            enter(State::GRIPPER_OPEN);
            return;
        }

        case State::GRIPPER_OPEN:
            if (!arm_->servo_gripper(kGripperOpenDeg)) {
                fail("gripper open failed");
                return;
            }
            enter(State::DESCEND);
            return;

        case State::DESCEND: {
            float x, y, z, r, p, yw;
            {
                std::lock_guard<std::mutex> lk(mu_);
                x = tgt_x_;
                y = tgt_y_;
                z = tgt_z_;
                r = tgt_roll_;
                p = tgt_pitch_;
                yw = tgt_yaw_;
            }
            if (!arm_->move_pose6d(x, y, z, r, p, yw, 0.5)) {
                fail("descend move failed");
                return;
            }
            enter(State::GRASP);
            return;
        }

        case State::GRASP:
            if (!arm_->servo_gripper(kGripperCloseDeg)) {
                fail("gripper close failed");
                return;
            }
            enter(State::LIFT);
            return;

        case State::LIFT: {
            float x, y, r, p, yw;
            {
                std::lock_guard<std::mutex> lk(mu_);
                x = tgt_x_;
                y = tgt_y_;
                r = tgt_roll_;
                p = tgt_pitch_;
                yw = tgt_yaw_;
            }
            if (!arm_->move_pose6d(x, y, kLiftZ, r, p, yw)) {
                fail("lift move failed");
                return;
            }
            enter(State::RETREAT);
            return;
        }

        case State::RETREAT: {
            if (!arm_->move_pose6d(0.0, 0.0, kRetreatZ, 180.0, 0.0, 0.0)) {
                fail("retreat move failed");
                return;
            }
            {
                std::lock_guard<std::mutex> lk(mu_);
                ++successes_;
                last_error_.clear();
                state_ = State::DONE;
            }
            std::fprintf(stderr, "proc_grasp: state -> done\n");
            return;
        }
    }
}
