#include "ctrl_client.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

CtrlClient::CtrlClient()  = default;
CtrlClient::~CtrlClient() { close(); }

bool CtrlClient::open(const std::string &target_path) {
    close();
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::fprintf(stderr, "proc_gateway: ctrl socket: %s\n", std::strerror(errno));
        return false;
    }
    target_ = target_path;
    return true;
}

void CtrlClient::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool CtrlClient::send_cmd(uint16_t cmd,
                           int32_t  a0, int32_t a1,
                           float    f0, float   f1) {
    if (fd_ < 0) return false;

    UavIpcHeader hdr{};
    hdr.magic        = UAV_IPC_MAGIC;
    hdr.version      = UAV_ABI_VERSION;
    hdr.type         = UAV_IPC_MSG_CTRL;
    hdr.payload_size = sizeof(UavCtrlPayload);

    UavCtrlPayload pay{};
    pay.cmd      = cmd;
    pay.i32_arg0 = a0;
    pay.i32_arg1 = a1;
    pay.f32_arg0 = f0;
    pay.f32_arg1 = f1;

    uint8_t buf[sizeof(hdr) + sizeof(pay)];
    std::memcpy(buf,              &hdr, sizeof(hdr));
    std::memcpy(buf + sizeof(hdr), &pay, sizeof(pay));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", target_.c_str());

    return sendto(fd_, buf, sizeof(buf), 0,
                  (sockaddr*)&addr, sizeof(addr)) == (ssize_t)sizeof(buf);
}
