#include "npu_application.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include "abi/ipc_framing.h"
#include "ctrl_server.h"
#include "npu_pipeline.h"
#include "npu_infer.h"
#include "result_publisher.h"
#include "shm_reader.h"

namespace {
const char *strategy_model_name(int32_t id) {
    switch (id) {
        case UAV_STRATEGY_BATTERY_V2: return "battery_v2.rknn";
        case UAV_STRATEGY_CUSTOM:     return "custom.rknn";
        case UAV_STRATEGY_DEFAULT:
        default:                      return "default.rknn";
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
    std::atomic<float> threshold{0.5F};
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

            if (!infer_enabled.load() || !infer.loaded()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
