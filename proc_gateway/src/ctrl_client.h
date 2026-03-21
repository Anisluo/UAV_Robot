#ifndef UAV_PROC_GATEWAY_CTRL_CLIENT_H
#define UAV_PROC_GATEWAY_CTRL_CLIENT_H

#include <cstdint>
#include <string>

#include "abi/ipc_framing.h"
#include "abi/msg_types.h"

// Sends UavCtrlPayload datagrams to a Unix-socket path (proc_realsense ctrl).
class CtrlClient {
public:
    CtrlClient();
    ~CtrlClient();

    bool open(const std::string &target_path);
    void close();

    bool send_cmd(uint16_t cmd,
                  int32_t  i32_arg0 = 0,
                  int32_t  i32_arg1 = 0,
                  float    f32_arg0 = 0.0F,
                  float    f32_arg1 = 0.0F);

private:
    int         fd_{-1};
    std::string target_;
};

#endif
