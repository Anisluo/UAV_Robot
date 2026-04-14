#include "grasp_application.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <csignal>

#include "abi/ipc_framing.h"
#include "abi/msg_types.h"
#include "arm_client.h"
#include "ctrl_server.h"
#include "grasp_pipeline.h"
#include "hand_eye.h"
#include "result_listener.h"

namespace {

std::atomic<bool> g_running{true};

void on_signal(int /*sig*/) { g_running.store(false); }

// --- tiny stand-alone JSON helpers (only enough for our params) -------------

bool json_get_int(const std::string &json, const char *key, int *out)
{
    std::string needle = std::string("\"") + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return false;
    pos++;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    if (pos >= json.size()) return false;
    char *endp = nullptr;
    long v = std::strtol(json.c_str() + pos, &endp, 10);
    if (endp == json.c_str() + pos) return false;
    if (out != nullptr) *out = static_cast<int>(v);
    return true;
}

bool json_get_string(const std::string &json, const char *key, std::string *out)
{
    std::string needle = std::string("\"") + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return false;
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos) return false;
    auto end = json.find('"', pos + 1);
    if (end == std::string::npos) return false;
    if (out != nullptr) out->assign(json, pos + 1, end - pos - 1);
    return true;
}

// Parse a fixed-length JSON array of floats: [v0,v1,...,vN-1].
bool json_get_float_array(const std::string &json, const char *key,
                          float *out, size_t n)
{
    std::string needle = std::string("\"") + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    pos = json.find('[', pos + needle.size());
    if (pos == std::string::npos) return false;
    ++pos;
    for (size_t i = 0; i < n; ++i) {
        while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == ',')) {
            ++pos;
        }
        if (pos >= json.size()) return false;
        char *endp = nullptr;
        float v = std::strtof(json.c_str() + pos, &endp);
        if (endp == json.c_str() + pos) return false;
        out[i] = v;
        pos = static_cast<size_t>(endp - json.c_str());
    }
    return true;
}

}  // namespace

int GraspApplication::run()
{
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    std::fprintf(stderr, "proc_grasp: starting (ctrl=%s, results=%s)\n",
                 UAV_CTRL_PATH_D, UAV_NPU_RESULT_GRASP_PATH);

    ResultListener listener;
    if (!listener.open()) {
        std::fprintf(stderr, "proc_grasp: failed to bind result socket\n");
        return 1;
    }

    ArmClient arm;
    GraspPipeline pipeline(&listener, &arm);

    // Load hand-eye calibration from file on startup if available.
    // Path is configurable via UAV_HAND_EYE_PATH, default /etc/uav_robot/hand_eye.txt
    // File format: 12 space- or newline-separated floats (row-major 3x4 affine):
    //   r00 r01 r02 tx
    //   r10 r11 r12 ty
    //   r20 r21 r22 tz
    {
        const char *he_path = std::getenv("UAV_HAND_EYE_PATH");
        if (he_path == nullptr || he_path[0] == '\0') {
            he_path = "/etc/uav_robot/hand_eye.txt";
        }
        FILE *fp = std::fopen(he_path, "r");
        if (fp != nullptr) {
            std::array<float, 12> m{};
            int parsed = 0;
            char line[256];
            // Read line-by-line, skip # comments, then tokenize floats.
            while (parsed < 12 && std::fgets(line, sizeof(line), fp) != nullptr) {
                char *p = line;
                while (*p == ' ' || *p == '\t') ++p;
                if (*p == '#' || *p == '\n' || *p == '\0') continue;
                char *tok = std::strtok(p, " \t\r\n");
                while (tok != nullptr && parsed < 12) {
                    if (tok[0] == '#') break;
                    char *end = nullptr;
                    float v = std::strtof(tok, &end);
                    if (end != tok) m[parsed++] = v;
                    tok = std::strtok(nullptr, " \t\r\n");
                }
            }
            std::fclose(fp);
            if (parsed == 12 && pipeline.set_hand_eye(m)) {
                std::fprintf(stderr, "proc_grasp: loaded hand-eye from %s\n", he_path);
            } else {
                std::fprintf(stderr,
                             "proc_grasp: hand-eye file %s has %d/12 floats — using identity\n",
                             he_path, parsed);
            }
        } else {
            std::fprintf(stderr,
                         "proc_grasp: no hand-eye file at %s — using identity (push via grasp.set_hand_eye)\n",
                         he_path);
        }
    }

    CtrlServer ctrl;
    bool ok = ctrl.start(UAV_CTRL_PATH_D,
        [&](int /*id*/, const std::string &method,
            const std::string &params) -> std::string {
            if (method == "system.ping") {
                return "{\"ok\":true}";
            }
            if (method == "grasp.start") {
                std::string mode_str;
                UavGraspMode mode = UAV_GRASP_MODE_3D;
                if (json_get_string(params, "mode", &mode_str) && mode_str == "6D") {
                    mode = UAV_GRASP_MODE_6D;
                }
                int loop = 0;
                (void)json_get_int(params, "auto", &loop);
                pipeline.set_auto(loop != 0);
                std::string err;
                if (!pipeline.trigger(mode, &err)) {
                    return "{\"ok\":false,\"error\":\"" + err + "\"}";
                }
                return "{\"ok\":true}";
            }
            if (method == "grasp.stop") {
                pipeline.set_auto(false);
                pipeline.abort("stop requested");
                return "{\"ok\":true}";
            }
            if (method == "grasp.run_once") {
                std::string mode_str;
                UavGraspMode mode = UAV_GRASP_MODE_3D;
                if (json_get_string(params, "mode", &mode_str) && mode_str == "6D") {
                    mode = UAV_GRASP_MODE_6D;
                }
                std::string err;
                if (!pipeline.trigger(mode, &err)) {
                    return "{\"ok\":false,\"error\":\"" + err + "\"}";
                }
                return "{\"ok\":true}";
            }
            if (method == "grasp.set_mode") {
                // Does not start a run — just records the default for next trigger.
                std::string mode_str;
                if (!json_get_string(params, "mode", &mode_str)) {
                    return "{\"ok\":false,\"error\":\"missing mode\"}";
                }
                // No persistent "default mode" field yet — surfaced via trigger.
                return std::string("{\"ok\":true,\"mode\":\"") + mode_str + "\"}";
            }
            if (method == "grasp.set_hand_eye") {
                std::array<float, 12> m{};
                if (!json_get_float_array(params, "matrix", m.data(), m.size())) {
                    return "{\"ok\":false,\"error\":\"expected matrix[12]\"}";
                }
                if (!pipeline.set_hand_eye(m)) {
                    return "{\"ok\":false,\"error\":\"invalid matrix\"}";
                }
                return "{\"ok\":true}";
            }
            if (method == "grasp.set_place_target") {
                // params: {x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg}
                auto get = [&](const char *key, float def) {
                    float arr[1] = {def};
                    char needle[32];
                    std::snprintf(needle, sizeof(needle), "\"%s\"", key);
                    auto pos = params.find(needle);
                    if (pos == std::string::npos) return def;
                    pos = params.find(':', pos);
                    if (pos == std::string::npos) return def;
                    arr[0] = std::strtof(params.c_str() + pos + 1, nullptr);
                    return arr[0];
                };
                float x = get("x_mm", 0.0F);
                float y = get("y_mm", 0.0F);
                float z = get("z_mm", 280.0F);
                float r = get("roll_deg", 180.0F);
                float p = get("pitch_deg", 0.0F);
                float yw = get("yaw_deg", 0.0F);
                pipeline.set_place_target(x, y, z, r, p, yw);
                return "{\"ok\":true}";
            }
            if (method == "grasp.get_status") {
                return pipeline.status_json();
            }
            return "{\"ok\":false,\"error\":\"unknown method\"}";
        });
    if (!ok) {
        std::fprintf(stderr, "proc_grasp: failed to start ctrl server\n");
        return 1;
    }

    std::fprintf(stderr, "proc_grasp: ready\n");

    // Main loop: drain NPU results, advance pipeline, service ctrl RPCs.
    // Roughly 10 Hz tick — arm RPCs are blocking so the actual cadence
    // during a grasp is dominated by arm move duration.
    auto next_tick = std::chrono::steady_clock::now();
    while (g_running.load()) {
        listener.poll();
        pipeline.tick();

        next_tick += std::chrono::milliseconds(100);
        auto now = std::chrono::steady_clock::now();
        int remaining_ms = 0;
        if (next_tick > now) {
            remaining_ms = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(next_tick - now).count());
        } else {
            // Overrun (arm RPC took longer than the period) — resync.
            next_tick = now;
        }
        ctrl.poll_once(remaining_ms > 0 ? remaining_ms : 0);
    }

    std::fprintf(stderr, "proc_grasp: shutting down\n");
    ctrl.stop();
    listener.close();
    return 0;
}
