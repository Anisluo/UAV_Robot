#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "abi/ipc_framing.h"
#include "ctrl_server.h"
#include "health.h"
#include "npu_infer.h"
#include "postprocess.h"
#include "preprocess.h"
#include "result_publisher.h"
#include "shm_reader.h"

// Map UavStrategyId to model filename
static const char *strategy_model_name(int32_t id) {
    switch (id) {
        case UAV_STRATEGY_BATTERY_V2: return "battery_v2.rknn";
        case UAV_STRATEGY_CUSTOM:     return "custom.rknn";
        case UAV_STRATEGY_DEFAULT:
        default:                      return "default.rknn";
    }
}

namespace {
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

void print_result(const UavCResult &r) {
    std::printf("C_RESULT frame=%llu det=%u\n",
                static_cast<unsigned long long>(r.frame_id),
                r.num_detections);
}
}

int main() {
    std::atomic<bool> running{true};
    std::atomic<bool> infer_enabled{true};
    std::atomic<float> threshold{0.5F};
    std::atomic<uint32_t> rate_fps{30U};
    // pending_model: protected by model_mu, empty means no reload needed
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

    ctrl.start(UAV_CTRL_PATH_C, [&](const UavCtrlPayload &cmd) {
        switch (cmd.cmd) {
            case UAV_CTRL_C_START:
                infer_enabled.store(true);
                break;
            case UAV_CTRL_C_STOP:
                infer_enabled.store(false);
                break;
            case UAV_CTRL_C_LOAD_MODEL:
                (void)infer.load_model("default.rknn");
                break;
            case UAV_CTRL_C_UNLOAD_MODEL:
                infer.unload_model();
                break;
            case UAV_CTRL_C_SET_THRESHOLD:
                if (cmd.f32_arg0 >= 0.0F && cmd.f32_arg0 <= 1.0F) {
                    threshold.store(cmd.f32_arg0);
                }
                break;
            case UAV_CTRL_C_SET_RATE:
                if (cmd.i32_arg0 > 0) {
                    rate_fps.store(static_cast<uint32_t>(cmd.i32_arg0));
                }
                break;
            case UAV_CTRL_C_SET_STRATEGY: {
                const char *name = strategy_model_name(cmd.i32_arg0);
                std::lock_guard<std::mutex> lk(model_mu);
                pending_model = name;
                std::printf("C_STATUS state=%d error=0\n",
                            static_cast<int>(UAV_PROC_STATE_IDLE)); // reloading
                break;
            }
            default:
                break;
        }
    });

    std::mutex latest_mu;
    InferenceFrame latest{};
    bool has_latest = false;

    CHealth health;
    Preprocess preprocess;
    Postprocess postprocess;

    std::thread ingest([&]() {
        while (running.load()) {
            InferenceFrame f{};
            if (!reader.wait_and_read(f, 100)) {
                continue;
            }
            std::lock_guard<std::mutex> lk(latest_mu);
            latest = std::move(f);
            has_latest = true;
        }
    });

    std::thread infer_thread([&]() {
        auto last_infer = std::chrono::steady_clock::now();
        while (running.load()) {
            // Check for pending model reload (strategy change)
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

            InferenceFrame f{};
            {
                std::lock_guard<std::mutex> lk(latest_mu);
                if (!has_latest) {
                    f.payload.clear();
                } else {
                    f = latest;
                    has_latest = false;
                }
            }
            if (f.payload.empty()) {
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

            const auto t0 = std::chrono::steady_clock::now();
            TensorInput tensor{};
            (void)preprocess.run(f, tensor);
            std::vector<RawDet> raw;
            if (!infer.infer(tensor, raw)) {
                continue;
            }
            UavCResult result = postprocess.run(f, raw, threshold.load());
            print_result(result);

            // Publish result to proc_gateway and app via Unix sockets
            publisher.publish(result);

            const auto t1 = std::chrono::steady_clock::now();
            const float lat_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            health.on_infer_done(lat_ms);
            const uint64_t ts = now_ns();
            if (health.should_emit_heartbeat(ts)) {
                print_heartbeat(health.make_heartbeat(ts));
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
