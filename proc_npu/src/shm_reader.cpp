#include "shm_reader.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
uint32_t load_state(const uint32_t *state) {
    return __atomic_load_n(state, __ATOMIC_ACQUIRE);
}

void store_state(uint32_t *state, uint32_t v) {
    __atomic_store_n(state, v, __ATOMIC_RELEASE);
}
}

ShmReader::ShmReader() : shm_fd_(-1), notify_fd_(-1), map_size_(0), ring_(nullptr) {}

ShmReader::~ShmReader() {
    close();
}

bool ShmReader::open_existing(const std::string &shm_name) {
    close();
    shm_fd_ = shm_open(shm_name.c_str(), O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::fprintf(stderr, "proc_npu: shm_open failed: %s\n", std::strerror(errno));
        return false;
    }

    struct stat st{};
    if (fstat(shm_fd_, &st) != 0) {
        std::fprintf(stderr, "proc_npu: fstat failed: %s\n", std::strerror(errno));
        close();
        return false;
    }
    map_size_ = static_cast<size_t>(st.st_size);
    void *addr = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::fprintf(stderr, "proc_npu: mmap failed: %s\n", std::strerror(errno));
        close();
        return false;
    }
    ring_ = static_cast<ShmRing *>(addr);
    if (ring_->magic != UAV_RING_MAGIC || ring_->version != UAV_RING_VERSION) {
        std::fprintf(stderr, "proc_npu: invalid ring header\n");
        close();
        return false;
    }
    return bind_notify_socket();
}

bool ShmReader::bind_notify_socket() {
    notify_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (notify_fd_ < 0) {
        std::fprintf(stderr, "proc_npu: notify socket failed: %s\n", std::strerror(errno));
        return false;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", UAV_RS_FRAME_NOTIFY_PATH);
    unlink(UAV_RS_FRAME_NOTIFY_PATH);
    if (bind(notify_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "proc_npu: notify bind failed: %s\n", std::strerror(errno));
        return false;
    }
    return true;
}

void ShmReader::close() {
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
        unlink(UAV_RS_FRAME_NOTIFY_PATH);
    }
    map_size_ = 0;
}

bool ShmReader::read_one_notify(UavFrameReadyPayload &notify, int timeout_ms) {
    pollfd pfd{};
    pfd.fd = notify_fd_;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN)) {
        return false;
    }

    uint8_t buffer[sizeof(UavIpcHeader) + sizeof(UavFrameReadyPayload)]{};
    const ssize_t n = recv(notify_fd_, buffer, sizeof(buffer), 0);
    if (n < static_cast<ssize_t>(sizeof(UavIpcHeader) + sizeof(UavFrameReadyPayload))) {
        return false;
    }

    UavIpcHeader hdr{};
    std::memcpy(&hdr, buffer, sizeof(hdr));
    if (hdr.magic != UAV_IPC_MAGIC || hdr.type != UAV_IPC_MSG_FRAME_READY ||
        hdr.payload_size != sizeof(UavFrameReadyPayload)) {
        return false;
    }
    std::memcpy(&notify, buffer + sizeof(hdr), sizeof(notify));
    return true;
}

bool ShmReader::claim_ready_slot(uint32_t slot_idx, InferenceFrame &frame) {
    if (ring_ == nullptr || slot_idx >= ring_->slot_count) {
        return false;
    }
    FrameSlot *slot = &ring_->slots[slot_idx];
    if (load_state(&slot->state) != UAV_SLOT_READY) {
        return false;
    }
    store_state(&slot->state, UAV_SLOT_READING);
    frame.slot = *slot;
    const uint8_t *src = shm_ring_slot_payload_const(ring_, slot_idx);
    frame.payload.assign(src, src + slot->payload_size);
    store_state(&slot->state, UAV_SLOT_FREE);
    return true;
}

bool ShmReader::wait_and_read(InferenceFrame &frame, int timeout_ms) {
    if (ring_ == nullptr) {
        return false;
    }

    // proc_realsense's writer sends a bare 8-byte "frame ready" signal
    // (no UavIpcHeader / no slot id), so the original read_one_notify
    // path size-mismatched and rejected every notification — leaving
    // NPU stuck in the stub while real frames flowed past. Drain any
    // pending wake-ups (size-agnostic) just to clear the socket queue,
    // then scan the ring directly for the newest READY slot, the same
    // way proc_gateway's reader does.
    if (notify_fd_ >= 0) {
        pollfd pfd{};
        pfd.fd = notify_fd_;
        pfd.events = POLLIN;
        if (poll(&pfd, 1, timeout_ms) > 0 && (pfd.revents & POLLIN)) {
            uint8_t scratch[64];
            while (recv(notify_fd_, scratch, sizeof(scratch), MSG_DONTWAIT) > 0) {
                // discard payload regardless of format / size
            }
        } else {
            // No fresh notification within timeout — but a slot may
            // still be READY from before. Fall through and scan once.
        }
    }

    const uint32_t N = ring_->slot_count;
    uint64_t best_fid = 0;
    uint32_t best_idx = N;
    for (uint32_t i = 0; i < N; ++i) {
        if (load_state(&ring_->slots[i].state) != UAV_SLOT_READY) continue;
        const uint64_t fid = ring_->slots[i].frame_id;
        if (fid > best_fid) { best_fid = fid; best_idx = i; }
    }
    if (best_idx == N) return false;

    // Auto-recover when proc_realsense restarts and frame_id resets:
    // accept the new sequence instead of holding onto a stale cursor.
    constexpr uint64_t kBackwardSlack = 1000;
    if (best_fid + kBackwardSlack < last_frame_id_) {
        last_frame_id_ = 0;
    }
    if (best_fid <= last_frame_id_) return false;
    last_frame_id_ = best_fid;
    // Non-destructive read: copy the slot's payload while it is still
    // READY, but do not flip the state machine to READING / FREE.
    // proc_gateway also consumes from this ring (and is the high-rate
    // reader feeding HostGUI's video stream); if NPU claimed every
    // slot, gateway would see them all as FREE and the JPEG push fps
    // would collapse from ~30 to <2. proc_realsense's writer reuses
    // any READY slot whose frame_id is the oldest, so leaving slots
    // marked READY is safe — the ring buffer continues to cycle.
    if (best_idx >= ring_->slot_count) return false;
    FrameSlot *slot = &ring_->slots[best_idx];
    frame.slot = *slot;
    const uint8_t *src = shm_ring_slot_payload_const(ring_, best_idx);
    frame.payload.assign(src, src + slot->payload_size);
    return true;
}
