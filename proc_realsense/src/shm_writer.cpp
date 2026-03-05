#include "shm_writer.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
bool store_state(uint32_t *state, uint32_t v) {
    __atomic_store_n(state, v, __ATOMIC_RELEASE);
    return true;
}

uint32_t load_state(const uint32_t *state) {
    return __atomic_load_n(state, __ATOMIC_ACQUIRE);
}
}

ShmWriter::ShmWriter()
    : shm_fd_(-1), notify_fd_(-1), map_size_(0), ring_(nullptr), drop_count_(0) {}

ShmWriter::~ShmWriter() {
    close();
}

bool ShmWriter::open_or_create(const std::string &shm_name, uint32_t slot_count, uint32_t slot_payload_size) {
    close();
    if (slot_count == 0 || slot_count > UAV_RING_MAX_SLOTS) {
        return false;
    }

    map_size_ = shm_ring_total_size(slot_count, slot_payload_size);
    shm_fd_ = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::fprintf(stderr, "proc_realsense: shm_open failed: %s\n", std::strerror(errno));
        return false;
    }
    if (ftruncate(shm_fd_, static_cast<off_t>(map_size_)) != 0) {
        std::fprintf(stderr, "proc_realsense: ftruncate failed: %s\n", std::strerror(errno));
        close();
        return false;
    }

    void *addr = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::fprintf(stderr, "proc_realsense: mmap failed: %s\n", std::strerror(errno));
        close();
        return false;
    }

    ring_ = static_cast<ShmRing *>(addr);
    ring_->magic = UAV_RING_MAGIC;
    ring_->version = UAV_RING_VERSION;
    ring_->slot_count = slot_count;
    ring_->slot_payload_size = slot_payload_size;
    ring_->write_index = 0;
    for (uint32_t i = 0; i < UAV_RING_MAX_SLOTS; ++i) {
        std::memset(&ring_->slots[i], 0, sizeof(FrameSlot));
        ring_->slots[i].state = UAV_SLOT_FREE;
    }

    notify_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (notify_fd_ < 0) {
        std::fprintf(stderr, "proc_realsense: notify socket failed: %s\n", std::strerror(errno));
        close();
        return false;
    }
    return true;
}

void ShmWriter::close() {
    if (ring_ != nullptr) {
        munmap(ring_, map_size_);
        ring_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    if (notify_fd_ >= 0) {
        ::close(notify_fd_);
        notify_fd_ = -1;
    }
    map_size_ = 0;
}

bool ShmWriter::notify_frame_ready(uint32_t slot_id, uint64_t frame_id, uint64_t timestamp_ns) {
    if (notify_fd_ < 0) {
        return false;
    }
    UavIpcHeader hdr{};
    hdr.magic = UAV_IPC_MAGIC;
    hdr.version = UAV_ABI_VERSION;
    hdr.type = UAV_IPC_MSG_FRAME_READY;
    hdr.payload_size = sizeof(UavFrameReadyPayload);

    UavFrameReadyPayload payload{};
    payload.slot_id = slot_id;
    payload.frame_id = frame_id;
    payload.timestamp_ns = timestamp_ns;

    uint8_t buffer[sizeof(UavIpcHeader) + sizeof(UavFrameReadyPayload)]{};
    std::memcpy(buffer, &hdr, sizeof(hdr));
    std::memcpy(buffer + sizeof(hdr), &payload, sizeof(payload));

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", UAV_DATA_NOTIFY_PATH_C);
    const ssize_t sent = sendto(
        notify_fd_,
        buffer,
        sizeof(buffer),
        MSG_DONTWAIT,
        reinterpret_cast<sockaddr *>(&addr),
        sizeof(addr));
    return sent == static_cast<ssize_t>(sizeof(buffer));
}

bool ShmWriter::write_frame(const CaptureFrame &frame) {
    if (ring_ == nullptr || ring_->slot_count == 0) {
        return false;
    }

    const uint32_t slot_count = ring_->slot_count;
    const uint32_t start = static_cast<uint32_t>(ring_->write_index % slot_count);
    for (uint32_t step = 0; step < slot_count; ++step) {
        const uint32_t idx = (start + step) % slot_count;
        FrameSlot *slot = &ring_->slots[idx];
        if (load_state(&slot->state) != UAV_SLOT_FREE) {
            continue;
        }
        store_state(&slot->state, UAV_SLOT_WRITING);

        const uint32_t color_size = static_cast<uint32_t>(frame.color.size());
        const uint32_t depth_size = static_cast<uint32_t>(frame.depth.size());
        const uint32_t total = color_size + depth_size;
        if (total > ring_->slot_payload_size) {
            store_state(&slot->state, UAV_SLOT_FREE);
            drop_count_++;
            return false;
        }

        uint8_t *dst = shm_ring_slot_payload(ring_, idx);
        std::memcpy(dst, frame.color.data(), color_size);
        std::memcpy(dst + color_size, frame.depth.data(), depth_size);

        slot->frame_id = frame.frame_id;
        slot->timestamp_ns = frame.timestamp_ns;
        slot->width = frame.width;
        slot->height = frame.height;
        slot->stride = frame.stride;
        slot->fx = frame.fx;
        slot->fy = frame.fy;
        slot->cx = frame.cx;
        slot->cy = frame.cy;
        slot->depth_scale = frame.depth_scale;
        slot->color_offset = 0;
        slot->depth_offset = color_size;
        slot->payload_size = total;

        store_state(&slot->state, UAV_SLOT_READY);
        ring_->write_index = frame.frame_id;
        (void)notify_frame_ready(idx, frame.frame_id, frame.timestamp_ns);
        return true;
    }

    drop_count_++;
    return false;
}

uint32_t ShmWriter::drop_count() const {
    return drop_count_;
}
