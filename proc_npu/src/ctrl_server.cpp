#include "ctrl_server.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

bool json_get_string(const std::string &json, const char *key, std::string &out) {
    std::string needle = std::string("\"") + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return false;
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos) return false;
    auto end = json.find('"', pos + 1);
    if (end == std::string::npos) return false;
    out = json.substr(pos + 1, end - pos - 1);
    return true;
}

bool json_get_int(const std::string &json, const char *key, int &out) {
    std::string needle = std::string("\"") + key + "\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return false;
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return false;
    pos++;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    if (pos >= json.size()) return false;
    char *endp = nullptr;
    out = static_cast<int>(std::strtol(json.c_str() + pos, &endp, 10));
    return endp != json.c_str() + pos;
}

std::string json_get_params(const std::string &json) {
    std::string needle = "\"params\"";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return "{}";
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) return "{}";
    pos++;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    if (pos >= json.size() || json[pos] != '{') return "{}";
    int depth = 0;
    auto start = pos;
    for (; pos < json.size(); ++pos) {
        if (json[pos] == '{') depth++;
        else if (json[pos] == '}') { depth--; if (depth == 0) { pos++; break; } }
    }
    return json.substr(start, pos - start);
}

static void set_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

} // namespace

CtrlServer::CtrlServer() : listen_fd_(-1), path_(), handler_(), clients_() {}

CtrlServer::~CtrlServer() {
    stop();
}

bool CtrlServer::start(const std::string &path, Handler handler) {
    stop();

    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::fprintf(stderr, "proc_npu: ctrl socket failed: %s\n", std::strerror(errno));
        return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
    unlink(path.c_str());

    if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "proc_npu: ctrl bind failed: %s\n", std::strerror(errno));
        stop();
        return false;
    }
    if (listen(listen_fd_, 4) != 0) {
        std::fprintf(stderr, "proc_npu: ctrl listen failed: %s\n", std::strerror(errno));
        stop();
        return false;
    }

    set_nonblock(listen_fd_);
    path_ = path;
    handler_ = std::move(handler);
    return true;
}

void CtrlServer::stop() {
    for (auto &c : clients_) {
        if (c.fd >= 0) ::close(c.fd);
    }
    clients_.clear();
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (!path_.empty()) {
        unlink(path_.c_str());
        path_.clear();
    }
}

void CtrlServer::poll_once(int timeout_ms) {
    if (listen_fd_ < 0) return;

    std::vector<pollfd> pfds;
    pfds.reserve(1 + clients_.size());
    pfds.push_back({listen_fd_, POLLIN, 0});
    for (auto &c : clients_) {
        pfds.push_back({c.fd, POLLIN, 0});
    }

    if (poll(pfds.data(), static_cast<nfds_t>(pfds.size()), timeout_ms) <= 0) {
        return;
    }

    if (pfds[0].revents & POLLIN) {
        accept_new();
    }

    for (std::size_t i = 0; i < clients_.size(); ) {
        if (pfds[i + 1].revents & (POLLIN | POLLHUP | POLLERR)) {
            handle_client(clients_[i]);
            if (clients_[i].fd < 0) {
                clients_.erase(clients_.begin() + static_cast<std::ptrdiff_t>(i));
                continue;
            }
        }
        ++i;
    }
}

void CtrlServer::accept_new() {
    int cfd = accept(listen_fd_, nullptr, nullptr);
    if (cfd < 0) return;
    set_nonblock(cfd);
    clients_.push_back({cfd, {}});
}

void CtrlServer::handle_client(Client &c) {
    char tmp[512];
    ssize_t n = recv(c.fd, tmp, sizeof(tmp) - 1, MSG_DONTWAIT);
    if (n <= 0) {
        close_client(c);
        return;
    }
    tmp[n] = '\0';
    c.buf.append(tmp, static_cast<std::size_t>(n));

    std::size_t start = 0;
    std::size_t nl;
    while ((nl = c.buf.find('\n', start)) != std::string::npos) {
        std::string line = c.buf.substr(start, nl - start);
        start = nl + 1;
        if (line.empty()) continue;

        int id = 0;
        std::string method;
        json_get_int(line, "id", id);
        json_get_string(line, "method", method);
        std::string params = json_get_params(line);

        std::string result = "{\"ok\":false,\"error\":\"unknown method\"}";
        if (handler_) {
            result = handler_(id, method, params);
        }
        send_response(c.fd, id, result);
    }
    c.buf.erase(0, start);
}

void CtrlServer::send_response(int fd, int id, const std::string &result_json) {
    char hdr[64];
    int hlen = std::snprintf(hdr, sizeof(hdr), "{\"id\":%d,\"result\":", id);
    std::string resp;
    resp.reserve(static_cast<std::size_t>(hlen) + result_json.size() + 3);
    resp.append(hdr, static_cast<std::size_t>(hlen));
    resp += result_json;
    resp += "}\n";
    (void)send(fd, resp.c_str(), resp.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
}

void CtrlServer::close_client(Client &c) {
    if (c.fd >= 0) {
        ::close(c.fd);
        c.fd = -1;
    }
}
