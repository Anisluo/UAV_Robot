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

    // Returns true if a new frame (frame_id > last seen) was consumed.
    bool read_latest(GatewayFrame &out);

private:
    int      fd_{-1};
    ShmRing *ring_{nullptr};
    size_t   map_size_{0};
    uint64_t last_frame_id_{0};
};

#endif
