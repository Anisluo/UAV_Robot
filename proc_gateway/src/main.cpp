// proc_gateway  –  bridges proc_realsense shm ring → TCP
//   :7001  JSON-RPC (newline-delimited)
//   :7002  MJPEG stream  [4-byte BE size][JPEG bytes]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "abi/ipc_framing.h"
#include "abi/msg_types.h"
#include "ctrl_client.h"
#include "jpeg_encoder.h"
#include "shm_reader.h"

// ─── NPU result receiver ─────────────────────────────────────────────────────
// Binds a Unix SOCK_DGRAM to UAV_NPU_RESULT_GW_PATH so proc_npu can push results.

struct NpuResultStore {
    std::mutex      mu;
    UavCResult      latest{};
    bool            has_result{false};
};

static NpuResultStore g_npu_result;

static int open_npu_result_socket() {
    ::unlink(UAV_NPU_RESULT_GW_PATH);
    int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, UAV_NPU_RESULT_GW_PATH,
                 sizeof(addr.sun_path) - 1);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd); return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

// Drain all pending datagrams from the npu result socket, cache the latest.
static void drain_npu_results(int fd) {
    if (fd < 0) return;
    UavCResult r{};
    ssize_t n;
    while ((n = ::recv(fd, &r, sizeof(r), MSG_DONTWAIT)) == static_cast<ssize_t>(sizeof(r))) {
        std::lock_guard<std::mutex> lk(g_npu_result.mu);
        g_npu_result.latest = r;
        g_npu_result.has_result = true;
    }
}

// ─── Task forwarding (UDP → uav_robotd --listen-port UAV_APP_CMD_PORT) ───────
static int open_task_udp_socket() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    return fd;
}

static void send_task_cmd(int fd, const char *cmd) {
    if (fd < 0) return;
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons(UAV_APP_CMD_PORT);
    ::sendto(fd, cmd, std::strlen(cmd), 0,
             reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));
}

// ─── Signal ──────────────────────────────────────────────────────────────────
static volatile sig_atomic_t g_running = 1;
static void on_sig(int) { g_running = 0; }

// ─── Time ────────────────────────────────────────────────────────────────────
static uint64_t now_ms() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch()).count();
}

// ─── TCP helpers ─────────────────────────────────────────────────────────────
static int listen_on(uint16_t port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    sockaddr_in a{};
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port        = htons(port);
    if (bind(fd, (sockaddr*)&a, sizeof(a)) || listen(fd, 8)) {
        close(fd); return -1;
    }
    return fd;
}

static void set_nonblock(int fd) {
    int f = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, f | O_NONBLOCK);
}

static void set_nodelay(int fd) {
    int opt = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
}

// ─── Minimal JSON field extractors ───────────────────────────────────────────
// These handle the simple, flat JSON objects sent by HostGUI.

static int json_int(const char *s, const char *key, int def = -1) {
    char pat[80];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(s, pat);
    if (!p) return def;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') ++p;
    char *end;
    long v = strtol(p, &end, 10);
    return (end != p) ? (int)v : def;
}

static std::string json_str(const char *s, const char *key) {
    char pat[80];
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(s, pat);
    if (!p) return {};
    p += strlen(pat);
    const char *e = strchr(p, '"');
    if (!e) return {};
    return std::string(p, e - p);
}

// ─── RPC connection state ─────────────────────────────────────────────────────
struct RpcConn {
    int         fd;
    std::string buf;
};

// Blocking write (RPC responses are tiny ≤ 256 bytes; EINTR/EAGAIN loop is fine)
static bool write_all(int fd, const char *data, size_t len) {
    while (len > 0) {
        ssize_t n = write(fd, data, len);
        if (n <= 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            return false;
        }
        data += n;
        len  -= n;
    }
    return true;
}

// ─── Build JSON for a single detection ───────────────────────────────────────
static int fmt_detection(char *buf, int bufsz, const UavDetection &d) {
    return snprintf(buf, (size_t)bufsz,
        "{\"class_id\":%d,\"score\":%.4f,"
        "\"x1\":%.1f,\"y1\":%.1f,\"x2\":%.1f,\"y2\":%.1f,"
        "\"x_mm\":%.2f,\"y_mm\":%.2f,\"z_mm\":%.2f,\"has_xyz\":%d}",
        d.class_id, d.score,
        d.x1, d.y1, d.x2, d.y2,
        d.x_mm, d.y_mm, d.z_mm,
        (int)d.has_xyz);
}

// ─── Handle one JSON-RPC request line ────────────────────────────────────────
static void handle_rpc(int fd, const std::string &line,
                        CtrlClient &ctrl_b, CtrlClient &ctrl_c,
                        int task_udp_fd,
                        uint64_t start_ms) {
    const char *s  = line.c_str();
    int         id = json_int(s, "id", 0);
    std::string method = json_str(s, "method");

    char resp[8192];

    if (method == "system.ping") {
        uint64_t uptime = now_ms() - start_ms;
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"pong\":true,\"uptime_ms\":%llu}}\n",
                 id, (unsigned long long)uptime);

    } else if (method == "camera.set_profile") {
        int w   = json_int(s, "width",  640);
        int h   = json_int(s, "height", 480);
        int fps = json_int(s, "fps",     30);
        ctrl_b.send_cmd(UAV_CTRL_B_SET_PROFILE, w, h, (float)fps);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "camera.set_exposure") {
        int us = json_int(s, "exposure_us", 0);
        ctrl_b.send_cmd(UAV_CTRL_B_SET_EXPOSURE, us);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    // ── NPU control ──────────────────────────────────────────────────────
    } else if (method == "npu.start") {
        ctrl_c.send_cmd(UAV_CTRL_C_START);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "npu.stop") {
        ctrl_c.send_cmd(UAV_CTRL_C_STOP);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "npu.set_strategy") {
        int strategy = json_int(s, "strategy_id", 0);
        ctrl_c.send_cmd(UAV_CTRL_C_SET_STRATEGY, strategy);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"strategy_id\":%d}}\n",
                 id, strategy);

    } else if (method == "npu.set_threshold") {
        // threshold is a float in params; parse via a simple float extraction
        const char *tp = strstr(s, "\"threshold\":");
        float thr = 0.5F;
        if (tp) {
            tp += strlen("\"threshold\":");
            while (*tp == ' ' || *tp == '\t') ++tp;
            char *end;
            thr = strtof(tp, &end);
            if (end == tp) thr = 0.5F;
        }
        ctrl_c.send_cmd(UAV_CTRL_C_SET_THRESHOLD, 0, 0, thr);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "npu.get_detections") {
        UavCResult r{};
        bool has = false;
        {
            std::lock_guard<std::mutex> lk(g_npu_result.mu);
            r   = g_npu_result.latest;
            has = g_npu_result.has_result;
        }
        if (!has) {
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"frame_id\":0,\"num_detections\":0,"
                     "\"detections\":[]}}\n", id);
        } else {
            // Build detections JSON array
            char dets_buf[6144];
            int  pos = 0;
            dets_buf[pos++] = '[';
            for (uint32_t i = 0; i < r.num_detections && i < UAV_MAX_DETECTIONS; ++i) {
                if (i > 0) dets_buf[pos++] = ',';
                pos += fmt_detection(dets_buf + pos,
                                     (int)(sizeof(dets_buf) - (size_t)pos),
                                     r.detections[i]);
            }
            dets_buf[pos++] = ']';
            dets_buf[pos]   = '\0';
            snprintf(resp, sizeof(resp),
                     "{\"id\":%d,\"result\":{\"frame_id\":%llu,"
                     "\"num_detections\":%u,\"detections\":%s}}\n",
                     id,
                     (unsigned long long)r.frame_id,
                     r.num_detections,
                     dets_buf);
        }

    } else if (method == "npu.get_status") {
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    // ── Task forwarding ───────────────────────────────────────────────────
    } else if (method == "task.start") {
        std::string task_name = json_str(s, "task");
        const char *cmd_str = "START_BATTERY_PICK";
        if (task_name == "arm_home")       cmd_str = "RESET";
        else if (task_name == "platform_lock") cmd_str = "START_BATTERY_PICK";
        send_task_cmd(task_udp_fd, cmd_str);
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true,\"task\":\"%s\"}}\n",
                 id, task_name.c_str());

    } else if (method == "task.stop") {
        send_task_cmd(task_udp_fd, "ESTOP");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else if (method == "task.reset") {
        send_task_cmd(task_udp_fd, "RESET");
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);

    } else {
        // Stub for arm/ugv/airport/gripper – acknowledge without hardware action
        snprintf(resp, sizeof(resp),
                 "{\"id\":%d,\"result\":{\"ok\":true}}\n", id);
    }

    write_all(fd, resp, strlen(resp));
}

// ─── Push one JPEG frame to a video client ───────────────────────────────────
// Returns false if the client should be closed.
static bool push_frame(int fd, const std::vector<uint8_t> &jpeg) {
    // Build header + payload as a single contiguous buffer for one send() call.
    uint32_t sz = (uint32_t)jpeg.size();
    std::vector<uint8_t> pkt(4 + jpeg.size());
    pkt[0] = (sz >> 24) & 0xFF;
    pkt[1] = (sz >> 16) & 0xFF;
    pkt[2] = (sz >>  8) & 0xFF;
    pkt[3] = (sz      ) & 0xFF;
    memcpy(pkt.data() + 4, jpeg.data(), jpeg.size());

    ssize_t n = send(fd, pkt.data(), pkt.size(), MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return true;    // buffer full – skip frame, keep client
        }
        return false;       // EPIPE / ECONNRESET → close
    }
    if ((size_t)n != pkt.size()) {
        return false;       // partial send corrupts framing → close
    }
    return true;
}

// ─── main ────────────────────────────────────────────────────────────────────
int main() {
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGPIPE, SIG_IGN);

    const uint64_t start_ms = now_ms();

    ShmReader   shm;
    JpegEncoder encoder;
    CtrlClient  ctrl_b;   // → proc_realsense
    CtrlClient  ctrl_c;   // → proc_npu

    ctrl_b.open(UAV_CTRL_PATH_B);
    ctrl_c.open(UAV_CTRL_PATH_C);

    // NPU result receiver socket
    int npu_result_fd = open_npu_result_socket();
    if (npu_result_fd < 0) {
        fprintf(stderr, "proc_gateway: warning – could not bind NPU result socket\n");
    }

    // Task command UDP socket (sends to uav_robotd)
    int task_udp_fd = open_task_udp_socket();

    int rpc_listen = listen_on(7001);
    int vid_listen = listen_on(7002);
    if (rpc_listen < 0 || vid_listen < 0) {
        fprintf(stderr, "proc_gateway: failed to bind ports 7001/7002\n");
        return 1;
    }
    fprintf(stderr, "proc_gateway: JSON-RPC :7001   MJPEG :7002\n");

    std::vector<RpcConn> rpc_clients;
    std::vector<int>     vid_clients;

    uint64_t last_shm_try = 0;
    GatewayFrame frame;

    while (g_running) {

        // ── Retry opening shm every 2 s while proc_realsense is not running ──
        if (!shm.is_open()) {
            uint64_t now = now_ms();
            if (now - last_shm_try >= 2000) {
                last_shm_try = now;
                if (shm.open(UAV_SHM_RING_NAME)) {
                    fprintf(stderr, "proc_gateway: shm ring opened\n");
                }
            }
        }

        // Drain any detection results pushed by proc_npu
        drain_npu_results(npu_result_fd);

        // ── Build pollfd: [0] rpc_listen, [1] vid_listen, [2+] rpc clients ──
        std::vector<pollfd> pfds;
        pfds.push_back({rpc_listen, POLLIN, 0});
        pfds.push_back({vid_listen, POLLIN, 0});
        for (auto &c : rpc_clients)
            pfds.push_back({c.fd, POLLIN, 0});

        int ret = poll(pfds.data(), (nfds_t)pfds.size(), 10 /*ms*/);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // ── Accept new RPC connection ─────────────────────────────────────────
        if (pfds[0].revents & POLLIN) {
            int fd = accept(rpc_listen, nullptr, nullptr);
            if (fd >= 0) {
                set_nodelay(fd);
                rpc_clients.push_back({fd, {}});
                fprintf(stderr, "proc_gateway: RPC client connected fd=%d\n", fd);
            }
        }

        // ── Accept new Video connection ───────────────────────────────────────
        if (pfds[1].revents & POLLIN) {
            int fd = accept(vid_listen, nullptr, nullptr);
            if (fd >= 0) {
                set_nonblock(fd);
                set_nodelay(fd);
                vid_clients.push_back(fd);
                fprintf(stderr, "proc_gateway: Video client connected fd=%d\n", fd);
            }
        }

        // ── Handle RPC client data ────────────────────────────────────────────
        std::vector<int> dead_rpc;
        for (size_t i = 0; i < rpc_clients.size(); ++i) {
            if (!(pfds[i + 2].revents & POLLIN)) continue;
            auto &c = rpc_clients[i];
            char tmp[4096];
            ssize_t n = read(c.fd, tmp, sizeof(tmp) - 1);
            if (n <= 0) {
                dead_rpc.push_back(c.fd);
                continue;
            }
            tmp[n] = '\0';
            c.buf += tmp;
            // Process complete newline-delimited JSON lines
            size_t pos;
            while ((pos = c.buf.find('\n')) != std::string::npos) {
                std::string line = c.buf.substr(0, pos);
                c.buf.erase(0, pos + 1);
                if (!line.empty())
                    handle_rpc(c.fd, line, ctrl_b, ctrl_c, task_udp_fd, start_ms);
            }
        }
        for (int dfd : dead_rpc) {
            rpc_clients.erase(
                std::remove_if(rpc_clients.begin(), rpc_clients.end(),
                    [dfd](const RpcConn &c){ return c.fd == dfd; }),
                rpc_clients.end());
            close(dfd);
            fprintf(stderr, "proc_gateway: RPC client disconnected\n");
        }

        // ── Read latest frame from shm → encode → push to video clients ──────
        if (shm.is_open() && shm.read_latest(frame) && !vid_clients.empty()) {
            std::vector<uint8_t> jpeg = encoder.encode(
                frame.color.data(), frame.width, frame.height, frame.stride);
            if (!jpeg.empty()) {
                std::vector<int> dead_vid;
                for (int vfd : vid_clients) {
                    if (!push_frame(vfd, jpeg))
                        dead_vid.push_back(vfd);
                }
                for (int dfd : dead_vid) {
                    vid_clients.erase(
                        std::remove(vid_clients.begin(), vid_clients.end(), dfd),
                        vid_clients.end());
                    close(dfd);
                    fprintf(stderr, "proc_gateway: Video client disconnected\n");
                }
            }
        }
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    for (auto &c : rpc_clients) close(c.fd);
    for (int fd  : vid_clients)  close(fd);
    close(rpc_listen);
    close(vid_listen);
    if (npu_result_fd >= 0) { close(npu_result_fd); ::unlink(UAV_NPU_RESULT_GW_PATH); }
    if (task_udp_fd   >= 0)   close(task_udp_fd);
    shm.close();
    ctrl_b.close();
    ctrl_c.close();
    fprintf(stderr, "proc_gateway: stopped\n");
    return 0;
}
