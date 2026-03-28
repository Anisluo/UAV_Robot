#include "realsense_application.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "abi/ipc_framing.h"
#include "abi/msg_types.h"
#include "ctrl_server.h"
#include "realsense_pipeline.h"
#include "rs_capture.h"
#include "shm_writer.h"

namespace {
uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

uint32_t env_u32(const char *name, uint32_t fallback) {
    const char *raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char *end = nullptr;
    unsigned long value = std::strtoul(raw, &end, 10);
    return (end != raw) ? static_cast<uint32_t>(value) : fallback;
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

int RealsenseApplication::run() {
    constexpr uint32_t kDefaultSlots = UAV_RING_DEFAULT_SLOTS;
    constexpr uint32_t kDefaultWidth = 424;
    constexpr uint32_t kDefaultHeight = 240;
    constexpr uint32_t kDefaultFps = 15;
    constexpr uint32_t kColorStride = kDefaultWidth * 3U;
    constexpr uint32_t kDepthStride = kDefaultWidth * 2U;
    constexpr uint32_t kSlotPayload = (kColorStride + kDepthStride) * kDefaultHeight;

    std::atomic<bool> running{true};
    std::atomic<bool> capture_enabled{true};
    const uint32_t initial_width = env_u32("UAV_REALSENSE_WIDTH", kDefaultWidth);
    const uint32_t initial_height = env_u32("UAV_REALSENSE_HEIGHT", kDefaultHeight);
    const uint32_t initial_fps = env_u32("UAV_REALSENSE_FPS", kDefaultFps);
    std::atomic<uint32_t> cfg_w{initial_width};
    std::atomic<uint32_t> cfg_h{initial_height};
    std::atomic<uint32_t> cfg_fps{initial_fps};

    RSCapture capture;
    ShmWriter writer;
    CtrlServer ctrl;
    RealsensePipeline pipeline(&writer);

    if (!writer.open_or_create(UAV_SHM_RING_NAME, kDefaultSlots, kSlotPayload)) {
        print_status(UAV_PROC_STATE_ERROR, -10);
        return 1;
    }
    capture.set_profile(cfg_w.load(), cfg_h.load(), cfg_fps.load());
    if (!capture.start()) {
        print_status(UAV_PROC_STATE_ERROR, -11);
        return 1;
    }

    ctrl.start(UAV_CTRL_PATH_B, [&](int /*id*/, const std::string &method,
                                    const std::string &params) -> std::string {
        (void)params;
        if (method == "realsense.start") {
            capture_enabled.store(true);
            return "{\"ok\":true}";
        }
        if (method == "realsense.stop") {
            capture_enabled.store(false);
            return "{\"ok\":true}";
        }
        if (method == "realsense.set_profile") {
            auto get_int = [&](const char *key, uint32_t fallback) -> uint32_t {
                auto pos = params.find(std::string("\"") + key + "\"");
                if (pos == std::string::npos) return fallback;
                pos = params.find(':', pos);
                if (pos == std::string::npos) return fallback;
                char *ep = nullptr;
                long value = std::strtol(params.c_str() + pos + 1, &ep, 10);
                return (ep != params.c_str() + pos + 1 && value > 0)
                    ? static_cast<uint32_t>(value)
                    : fallback;
            };
            cfg_w.store(get_int("width", cfg_w.load()));
            cfg_h.store(get_int("height", cfg_h.load()));
            cfg_fps.store(get_int("fps", cfg_fps.load()));
            capture.set_profile(cfg_w.load(), cfg_h.load(), cfg_fps.load());
            return "{\"ok\":true}";
        }
        if (method == "realsense.set_exposure") {
            auto pos = params.find("\"exposure\"");
            if (pos != std::string::npos) {
                pos = params.find(':', pos);
                if (pos != std::string::npos) {
                    char *ep = nullptr;
                    int value = static_cast<int>(std::strtol(params.c_str() + pos + 1, &ep, 10));
                    if (ep != params.c_str() + pos + 1) {
                        capture.set_exposure(value);
                    }
                }
            }
            return "{\"ok\":true}";
        }
        if (method == "realsense.snapshot") {
            CaptureFrame frame{};
            (void)capture.snapshot(frame);
            return "{\"ok\":true}";
        }
        if (method == "system.ping") {
            return "{\"ok\":true}";
        }
        return "{\"ok\":false,\"error\":\"unknown method\"}";
    });

    print_status(UAV_PROC_STATE_RUNNING, 0);
    while (running.load()) {
        ctrl.poll_once(1);
        if (!capture_enabled.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            UavBHeartbeat heartbeat{};
            pipeline.emit_idle_heartbeat(now_ns(), &heartbeat);
            if (heartbeat.timestamp_ns != 0U) {
                print_hb(heartbeat);
            }
            continue;
        }

        CaptureFrame frame{};
        if (!capture.snapshot(frame)) {
            print_status(UAV_PROC_STATE_ERROR, -12);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        UavBHeartbeat heartbeat{};
        if (pipeline.publish_frame(frame, now_ns(), &heartbeat) && heartbeat.timestamp_ns != 0U) {
            print_hb(heartbeat);
        }
    }

    ctrl.stop();
    capture.stop();
    writer.close();
    print_status(UAV_PROC_STATE_IDLE, 0);
    return 0;
}
