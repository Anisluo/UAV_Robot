#ifndef UAV_PROC_REALSENSE_CTRL_SERVER_H
#define UAV_PROC_REALSENSE_CTRL_SERVER_H

#include <cstdint>
#include <functional>
#include <string>

#include "abi/ipc_framing.h"

class CtrlServer {
public:
    using Handler = std::function<void(const UavCtrlPayload &)>;

    CtrlServer();
    ~CtrlServer();

    bool start(const std::string &path, Handler handler);
    void stop();
    void poll_once(int timeout_ms);

private:
    int fd_;
    std::string path_;
    Handler handler_;
};

#endif
