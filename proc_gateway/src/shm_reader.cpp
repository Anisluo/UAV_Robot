#include "shm_reader.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "abi/ipc_framing.h"

ShmReader::ShmReader()  = default;
ShmReader::~ShmReader() { close(); }

bool ShmReader::open(const std::string &shm_name) {
    close();
    fd_ = shm_open(shm_name.c_str(), O_RDWR, 0666);
    if (fd_ < 0) {
        return false;   // caller will retry silently
    }

    ShmRing hdr{};
    if (pread(fd_, &hdr, sizeof(hdr), 0) < (ssize_t)sizeof(hdr)) {
        ::close(fd_); fd_ = -1;
        return false;
    }
    if (hdr.magic != UAV_RING_MAGIC || hdr.slot_count == 0) {
        ::close(fd_); fd_ = -1;
        return false;
    }

    map_size_ = shm_ring_total_size(hdr.slot_count, hdr.slot_payload_size);
    void *addr = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (addr == MAP_FAILED) {
        std::fprintf(stderr, "proc_gateway: mmap failed: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }

    ring_          = static_cast<ShmRing *>(addr);
    last_frame_id_ = 0;
    return true;
}

void ShmReader::close() {
    if (ring_)           { munmap(ring_, map_size_); ring_ = nullptr; }
    if (fd_ >= 0)        { ::close(fd_); fd_ = -1; }
    if (notify_fd_ >= 0) { ::close(notify_fd_); notify_fd_ = -1; }
    map_size_ = 0;
}

int ShmReader::open_notify_fd() {
    if (notify_fd_ >= 0) return notify_fd_;

    notify_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (notify_fd_ < 0) {
        std::fprintf(stderr, "proc_gateway: frame notify socket failed: %s\n", std::strerror(errno));
        return -1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", UAV_RS_FRAME_NOTIFY_PATH);
    unlink(UAV_RS_FRAME_NOTIFY_PATH);

    if (bind(notify_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "proc_gateway: frame notify bind failed: %s\n", std::strerror(errno));
        ::close(notify_fd_);
        notify_fd_ = -1;
        return -1;
    }

    int flags = fcntl(notify_fd_, F_GETFL, 0);
    fcntl(notify_fd_, F_SETFL, flags | O_NONBLOCK);
    return notify_fd_;
}

void ShmReader::drain_notify_fd() {
    if (notify_fd_ < 0) return;
    uint64_t val;
    while (recv(notify_fd_, &val, sizeof(val), MSG_DONTWAIT) == static_cast<ssize_t>(sizeof(val))) {
        // 消费所有待读通知，防止 epoll 持续触发
    }
}

bool ShmReader::read_latest(GatewayFrame &out) {
    if (!ring_) return false;

    const uint32_t N = ring_->slot_count;

    // Find the READY slot with the highest frame_id
    uint64_t best_fid = 0;
    uint32_t best_idx = N;
    for (uint32_t i = 0; i < N; ++i) {
        if (__atomic_load_n(&ring_->slots[i].state, __ATOMIC_ACQUIRE) == UAV_SLOT_READY) {
            uint64_t fid = ring_->slots[i].frame_id;
            if (fid > best_fid) { best_fid = fid; best_idx = i; }
        }
    }

    if (best_idx == N)                return false;   // no READY slot
    if (best_fid <= last_frame_id_)   return false;   // nothing new

    FrameSlot *slot = &ring_->slots[best_idx];

    // Atomically claim READY → READING
    uint32_t expected = UAV_SLOT_READY;
    if (!__atomic_compare_exchange_n(&slot->state, &expected,
                                     (uint32_t)UAV_SLOT_READING,
                                     false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)) {
        return false;   // grabbed by someone else
    }

    out.frame_id = slot->frame_id;
    out.width    = slot->width;
    out.height   = slot->height;
    out.stride   = slot->stride;

    const uint32_t color_size = slot->stride * slot->height;
    const uint8_t *payload    = shm_ring_slot_payload_const(ring_, best_idx);
    out.color.assign(payload + slot->color_offset,
                     payload + slot->color_offset + color_size);

    __atomic_store_n(&slot->state, (uint32_t)UAV_SLOT_FREE, __ATOMIC_RELEASE);
    last_frame_id_ = out.frame_id;
    return true;
}
