#ifndef UAV_PROC_GATEWAY_SHM_READER_H
#define UAV_PROC_GATEWAY_SHM_READER_H

#include <cstdint>
#include <string>
#include <vector>

#include "abi/shm_ring.h"

struct GatewayFrame {
    uint64_t             frame_id{0};
    uint32_t             width{0};
    uint32_t             height{0};
    uint32_t             stride{0};   // bytes per color row (BGR8: width*3)
    std::vector<uint8_t> color;       // BGR8 pixels
};

class ShmReader {
public:
    ShmReader();
    ~ShmReader();

    bool open(const std::string &shm_name);
    void close();
    bool is_open() const { return ring_ != nullptr; }

    // 打开帧就绪通知 socket（绑定 UAV_RS_FRAME_NOTIFY_PATH）。
    // 返回 fd 供调用方加入 epoll/poll；fd 由 ShmReader 管理，close() 时自动关闭。
    // proc_realsense 每写完一帧发送 8 字节 uint64 到此 socket。
    int open_notify_fd();
    // 消费通知 fd 上的所有待读字节（epoll 回调后调用，防止 fd 一直可读）
    void drain_notify_fd();

    // Returns true if a new frame (frame_id > last seen) was consumed.
    bool read_latest(GatewayFrame &out);

private:
    int      fd_{-1};
    int      notify_fd_{-1};  // SOCK_DGRAM 接收帧就绪通知
    ShmRing *ring_{nullptr};
    size_t   map_size_{0};
    uint64_t last_frame_id_{0};
};

#endif
