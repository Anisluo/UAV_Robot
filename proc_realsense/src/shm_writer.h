#ifndef UAV_PROC_REALSENSE_SHM_WRITER_H
#define UAV_PROC_REALSENSE_SHM_WRITER_H

#include <cstdint>
#include <string>

#include "abi/ipc_framing.h"
#include "abi/shm_ring.h"
#include "rs_capture.h"

class ShmWriter {
public:
    ShmWriter();
    ~ShmWriter();

    bool open_or_create(const std::string &shm_name, uint32_t slot_count, uint32_t slot_payload_size);
    void close();
    bool write_frame(const CaptureFrame &frame);
    uint32_t drop_count() const;

private:
    bool notify_frame_ready();

    int shm_fd_;
    int notify_fd_;   // Unix SOCK_DGRAM，向 UAV_RS_FRAME_NOTIFY_PATH 发 8 字节计数
    size_t map_size_;
    ShmRing *ring_;
    uint32_t drop_count_;
};

#endif
