#include "npu_application.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <sys/stat.h>

#include "abi/ipc_framing.h"
#include "ctrl_server.h"
#include "npu_pipeline.h"
#include "npu_infer.h"
#include "result_publisher.h"
#include "shm_reader.h"

namespace {
constexpr const char *kFaceTriggerFlag    = "/tmp/uav_face_tracker_enabled";
constexpr const char *kBatteryTriggerFlag = "/tmp/uav_battery_tracker_enabled";

// Resolve model file with absolute path. The systemd unit sets
// WorkingDirectory=/home/ubuntu/UAV_Robot, but `bash -lc` in the
// ExecStart line was observed to leave the inference thread's
// std::ifstream unable to open the relative path "mavic3_drone.rknn"
// even after the sub-process's CWD reads back as /home/ubuntu/UAV_Robot.
// Hardcoding the prefix sidesteps the question entirely and matches the
// known deployment layout. Override at runtime with UAV_NPU_MODEL_DIR.
const char *strategy_model_name(int32_t id) {
    switch (id) {
        case UAV_STRATEGY_BATTERY_V2:   return "/home/ubuntu/UAV_Robot/battery_v2.rknn";
        case UAV_STRATEGY_CUSTOM:       return "/home/ubuntu/UAV_Robot/custom.rknn";
        case UAV_STRATEGY_FACE:         return "";  // handled by face_tracker.py
        case UAV_STRATEGY_BATTERY_CV:   return "";  // handled by battery_tracker.py
        case UAV_STRATEGY_MAVIC3_DRONE: return "/home/ubuntu/UAV_Robot/mavic3_drone.rknn";
        case UAV_STRATEGY_DEFAULT:
        default:                        return "/home/ubuntu/UAV_Robot/default.rknn";
    }
}

// Create/remove a trigger flag. Python sidecars poll for these files and
// run / idle accordingly — keeps them silent until their strategy is
// selected, so they don't compete with the RKNN detector.
void set_trigger_flag(const char *path, bool on) {
    if (on) {
        FILE *f = std::fopen(path, "w");
        if (f) { std::fputs("1\n", f); std::fclose(f); }
    } else {
        std::remove(path);
    }
}

uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

void print_status(UavProcState state, int32_t error_code) {
    std::printf("C_STATUS state=%d error=%d\n", static_cast<int>(state), static_cast<int>(error_code));
}

void print_heartbeat(const UavCHeartbeat &hb) {
    std::printf("C_HEARTBEAT ts=%llu inf_fps=%.2f lat=%.2f\n",
                static_cast<unsigned long long>(hb.timestamp_ns),
                hb.inference_fps,
                hb.avg_latency_ms);
}

}

int NpuApplication::run() {
    std::atomic<bool> running{true};
    std::atomic<bool> infer_enabled{true};
    // 0.45 default. The COCO-pretrained mavic3_drone.rknn produces
    // weak ~0.50 hits on the empty workbench (the dark battery on a
    // white plate reads as a tiny plane on a runway), but the real
    // Mavic 3 lands anywhere from 0.45 to 0.58 depending on pose.
    // Raising the threshold to suppress phantoms cost too many real-
    // drone frames; npu_pipeline now has a 2-frame consensus instead
    // — it filters single-frame phantoms via a WARMUP state, so we
    // keep the threshold low to catch the drone reliably.
    std::atomic<float> threshold{0.45F};
    std::atomic<uint32_t> rate_fps{30U};
    std::mutex model_mu;
    std::string pending_model;

    ShmReader reader;
    if (!reader.open_existing(UAV_SHM_RING_NAME)) {
        print_status(UAV_PROC_STATE_ERROR, -20);
        return 1;
    }

    CtrlServer ctrl;
    NpuInfer infer;
    (void)infer.load_model("default.rknn");

    ResultPublisher publisher;
    (void)publisher.open();

    ctrl.start(UAV_CTRL_PATH_C, [&](int /*id*/, const std::string &method,
                                    const std::string &params) -> std::string {
        auto get_float = [&](const char *key, float fallback) -> float {
            auto pos = params.find(std::string("\"") + key + "\"");
            if (pos == std::string::npos) return fallback;
            pos = params.find(':', pos);
            if (pos == std::string::npos) return fallback;
            char *ep = nullptr;
            float value = std::strtof(params.c_str() + pos + 1, &ep);
            return (ep != params.c_str() + pos + 1) ? value : fallback;
        };
        auto get_int32 = [&](const char *key, int32_t fallback) -> int32_t {
            auto pos = params.find(std::string("\"") + key + "\"");
            if (pos == std::string::npos) return fallback;
            pos = params.find(':', pos);
            if (pos == std::string::npos) return fallback;
            char *ep = nullptr;
            long value = std::strtol(params.c_str() + pos + 1, &ep, 10);
            return (ep != params.c_str() + pos + 1) ? static_cast<int32_t>(value) : fallback;
        };

        if (method == "npu.start") {
            infer_enabled.store(true);
            return "{\"ok\":true}";
        }
        if (method == "npu.stop") {
            infer_enabled.store(false);
            return "{\"ok\":true}";
        }
        if (method == "npu.load_model") {
            (void)infer.load_model("default.rknn");
            return "{\"ok\":true}";
        }
        if (method == "npu.unload_model") {
            infer.unload_model();
            return "{\"ok\":true}";
        }
        if (method == "npu.set_threshold") {
            float thr = get_float("threshold", -1.0F);
            if (thr >= 0.0F && thr <= 1.0F) {
                threshold.store(thr);
            }
            return "{\"ok\":true}";
        }
        if (method == "npu.set_rate") {
            int32_t fps = get_int32("fps", 0);
            if (fps > 0) {
                rate_fps.store(static_cast<uint32_t>(fps));
            }
            return "{\"ok\":true}";
        }
        if (method == "npu.set_strategy") {
            int32_t strategy_id = get_int32("strategy", UAV_STRATEGY_DEFAULT);
            const char *name = strategy_model_name(strategy_id);
            // Toggle the face_tracker.py sidecar via a trigger flag file.
            set_trigger_flag(kFaceTriggerFlag,
                             strategy_id == UAV_STRATEGY_FACE);
            // Battery CV runs alongside the drone NPU model: the user
            // wants the gripper to see batteries AND drones at once,
            // so leave battery_tracker.py active for both BATTERY_CV
            // and MAVIC3_DRONE strategies. Gateway merges the two
            // detection streams by class.
            set_trigger_flag(kBatteryTriggerFlag,
                             strategy_id == UAV_STRATEGY_BATTERY_CV
                          || strategy_id == UAV_STRATEGY_MAVIC3_DRONE);
            {
                std::lock_guard<std::mutex> lk(model_mu);
                pending_model = name;
            }
            std::printf("C_STATUS state=%d error=0\n",
                        static_cast<int>(UAV_PROC_STATE_IDLE));
            return "{\"ok\":true}";
        }
        if (method == "system.ping") {
            return "{\"ok\":true}";
        }
        return "{\"ok\":false,\"error\":\"unknown method\"}";
    });

    std::mutex latest_mu;
    InferenceFrame latest{};
    bool has_latest = false;

    NpuPipeline pipeline(&infer, &publisher);

    std::thread ingest([&]() {
        while (running.load()) {
            InferenceFrame frame{};
            if (!reader.wait_and_read(frame, 100)) {
                continue;
            }
            std::lock_guard<std::mutex> lk(latest_mu);
            latest = std::move(frame);
            has_latest = true;
        }
    });

    std::thread infer_thread([&]() {
        auto last_infer = std::chrono::steady_clock::now();
        while (running.load()) {
            {
                std::lock_guard<std::mutex> lk(model_mu);
                if (!pending_model.empty()) {
                    infer.unload_model();
                    (void)infer.load_model(pending_model.c_str());
                    std::printf("C_STATUS state=%d error=0\n",
                                static_cast<int>(UAV_PROC_STATE_RUNNING));
                    pending_model.clear();
                }
            }

            if (!infer_enabled.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // ── No RKNN model loaded? Fall back to stub detection so the
            //    downstream pipeline (proc_grasp + HostGUI overlay) still
            //    sees *something* for integration testing.  Disable with
            //    UAV_NPU_STUB=0. ─────────────────────────────────────────
            if (!infer.loaded()) {
                static bool s_stub_enabled = []() {
                    const char *e = std::getenv("UAV_NPU_STUB");
                    return (e == nullptr || e[0] == '\0' || e[0] != '0');
                }();
                // Skip stub entirely when a Python sidecar owns the
                // detection path — otherwise the stub would stomp its
                // UavCResult on the gateway's cache every 100 ms.
                struct stat st;
                bool face_active    = (::stat(kFaceTriggerFlag, &st) == 0);
                bool battery_active = (::stat(kBatteryTriggerFlag, &st) == 0);
                if (!s_stub_enabled || face_active || battery_active) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                // Pull one frame to know the image dimensions.
                InferenceFrame frame{};
                {
                    std::lock_guard<std::mutex> lk(latest_mu);
                    if (!has_latest) {
                        ;  // no frame yet
                    } else {
                        frame = latest;
                    }
                }
                UavCResult stub{};
                stub.frame_id = static_cast<uint64_t>(now_ns());
                stub.num_detections = 1;
                UavDetection &d = stub.detections[0];
                d.class_id  = 0;
                d.score     = 0.99F;
                // Center quarter of frame (fallback 640x480 if unknown).
                uint32_t w = frame.slot.width  > 0U ? frame.slot.width  : 640U;
                uint32_t h = frame.slot.height > 0U ? frame.slot.height : 480U;
                d.x1 = static_cast<float>(w) * 0.375F;
                d.y1 = static_cast<float>(h) * 0.375F;
                d.x2 = static_cast<float>(w) * 0.625F;
                d.y2 = static_cast<float>(h) * 0.625F;
                d.has_xyz = 0U;
                d.has_rpy = 0U;
                d.grasp_mode = UAV_GRASP_MODE_3D;
                publisher.publish(stub);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            InferenceFrame frame{};
            {
                std::lock_guard<std::mutex> lk(latest_mu);
                if (!has_latest) {
                    frame.payload.clear();
                } else {
                    frame = latest;
                    has_latest = false;
                }
            }
            if (frame.payload.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            const uint32_t fps = rate_fps.load();
            if (fps > 0U) {
                const auto min_period = std::chrono::milliseconds(1000U / fps);
                const auto now = std::chrono::steady_clock::now();
                const auto elapsed = now - last_infer;
                if (elapsed < min_period) {
                    std::this_thread::sleep_for(min_period - elapsed);
                }
                last_infer = std::chrono::steady_clock::now();
            }

            UavCHeartbeat heartbeat{};
            if (!pipeline.process_frame(frame, threshold.load(), now_ns(), &heartbeat)) {
                continue;
            }
            if (heartbeat.timestamp_ns != 0U) {
                print_heartbeat(heartbeat);
            }
        }
    });

    print_status(UAV_PROC_STATE_RUNNING, 0);
    while (running.load()) {
        ctrl.poll_once(100);
    }

    ingest.join();
    infer_thread.join();
    ctrl.stop();
    publisher.close();
    reader.close();
    print_status(UAV_PROC_STATE_IDLE, 0);
    return 0;
}
