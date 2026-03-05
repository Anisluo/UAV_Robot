#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include "abi/ipc_framing.h"
#include "abi/msg_types.h"
#include "ctrl_server.h"
#include "health.h"
#include "rs_capture.h"
#include "shm_writer.h"

namespace {
uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

void print_hb(const UavBHeartbeat &hb) {
    std::printf("B_HEARTBEAT ts=%llu fps=%.2f drop=%u\n",
                static_cast<unsigned long long>(hb.timestamp_ns),
                hb.fps,
                hb.drop_count);
}

void print_status(UavProcState state, int32_t error_code) {
    std::printf("B_STATUS state=%d error=%d\n", static_cast<int>(state), static_cast<int>(error_code));
}
}

int main() {
    constexpr uint32_t kDefaultSlots = UAV_RING_DEFAULT_SLOTS;
    constexpr uint32_t kDefaultWidth = 640;
    constexpr uint32_t kDefaultHeight = 480;
    constexpr uint32_t kColorStride = kDefaultWidth * 3U;
    constexpr uint32_t kDepthStride = kDefaultWidth * 2U;
    constexpr uint32_t kSlotPayload = (kColorStride + kDepthStride) * kDefaultHeight;

    std::atomic<bool> running{true};
    std::atomic<bool> capture_enabled{true};
    std::atomic<uint32_t> cfg_w{kDefaultWidth};
    std::atomic<uint32_t> cfg_h{kDefaultHeight};
    std::atomic<uint32_t> cfg_fps{30U};

    RSCapture capture;
    ShmWriter writer;
    BHealth health;
    CtrlServer ctrl;

    if (!writer.open_or_create(UAV_SHM_RING_NAME, kDefaultSlots, kSlotPayload)) {
        print_status(UAV_PROC_STATE_ERROR, -10);
        return 1;
    }
    if (!capture.start()) {
        print_status(UAV_PROC_STATE_ERROR, -11);
        return 1;
    }

    ctrl.start(UAV_CTRL_PATH_B, [&](const UavCtrlPayload &cmd) {
        switch (cmd.cmd) {
            case UAV_CTRL_B_START:
                capture_enabled.store(true);
                break;
            case UAV_CTRL_B_STOP:
                capture_enabled.store(false);
                break;
            case UAV_CTRL_B_SET_PROFILE:
                cfg_w.store(cmd.i32_arg0 > 0 ? static_cast<uint32_t>(cmd.i32_arg0) : cfg_w.load());
                cfg_h.store(cmd.i32_arg1 > 0 ? static_cast<uint32_t>(cmd.i32_arg1) : cfg_h.load());
                if (cmd.f32_arg0 > 0.0F) {
                    cfg_fps.store(static_cast<uint32_t>(cmd.f32_arg0));
                }
                capture.set_profile(cfg_w.load(), cfg_h.load(), cfg_fps.load());
                break;
            case UAV_CTRL_B_SET_EXPOSURE:
                capture.set_exposure(cmd.i32_arg0);
                break;
            case UAV_CTRL_B_SNAPSHOT: {
                CaptureFrame f{};
                (void)capture.snapshot(f);
                break;
            }
            default:
                break;
        }
    });

    print_status(UAV_PROC_STATE_RUNNING, 0);
    while (running.load()) {
        ctrl.poll_once(1);
        if (!capture_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (health.should_emit_heartbeat(now_ns())) {
                print_hb(health.make_heartbeat(now_ns()));
            }
            continue;
        }

        CaptureFrame frame{};
        if (!capture.snapshot(frame)) {
            print_status(UAV_PROC_STATE_ERROR, -12);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (writer.write_frame(frame)) {
            health.on_frame_published();
        }
        health.set_drop_count(writer.drop_count());
        const uint64_t ts = now_ns();
        if (health.should_emit_heartbeat(ts)) {
            print_hb(health.make_heartbeat(ts));
        }
    }

    ctrl.stop();
    capture.stop();
    writer.close();
    print_status(UAV_PROC_STATE_IDLE, 0);
    return 0;
}
