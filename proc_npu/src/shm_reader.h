#ifndef UAV_PROC_NPU_SHM_READER_H
#define UAV_PROC_NPU_SHM_READER_H

#include <cstdint>
#include <string>
#include <vector>

#include "abi/ipc_framing.h"
#include "abi/shm_ring.h"

struct InferenceFrame {
    FrameSlot slot;
    std::vector<uint8_t> payload;
};

class ShmReader {
public:
    ShmReader();
    ~ShmReader();

    bool open_existing(const std::string &shm_name);
    void close();
    bool wait_and_read(InferenceFrame &frame, int timeout_ms);

private:
    bool bind_notify_socket();
    bool read_one_notify(UavFrameReadyPayload &notify, int timeout_ms);
    bool claim_ready_slot(uint32_t slot_idx, InferenceFrame &frame);

    int shm_fd_;
    int notify_fd_;
    size_t map_size_;
    ShmRing *ring_;
};

#endif
