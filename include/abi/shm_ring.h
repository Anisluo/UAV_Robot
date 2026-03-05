#ifndef UAV_ABI_SHM_RING_H
#define UAV_ABI_SHM_RING_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UAV_RING_MAGIC 0x55565247U
#define UAV_RING_VERSION 1U
#define UAV_RING_DEFAULT_SLOTS 8U
#define UAV_RING_MAX_SLOTS 16U

typedef enum {
    UAV_SLOT_FREE = 0,
    UAV_SLOT_WRITING = 1,
    UAV_SLOT_READY = 2,
    UAV_SLOT_READING = 3
} UavSlotState;

typedef struct {
    uint32_t state;
    uint64_t frame_id;
    uint64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    float fx;
    float fy;
    float cx;
    float cy;
    float depth_scale;
    uint32_t color_offset;
    uint32_t depth_offset;
    uint32_t payload_size;
    uint32_t reserved;
} FrameSlot;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t slot_count;
    uint32_t slot_payload_size;
    uint64_t write_index;
    FrameSlot slots[UAV_RING_MAX_SLOTS];
    uint8_t payload[];
} ShmRing;

static inline size_t shm_ring_total_size(uint32_t slot_count, uint32_t slot_payload_size) {
    return sizeof(ShmRing) + ((size_t)slot_count * (size_t)slot_payload_size);
}

static inline uint8_t *shm_ring_slot_payload(ShmRing *ring, uint32_t slot_idx) {
    return ring->payload + ((size_t)slot_idx * (size_t)ring->slot_payload_size);
}

static inline const uint8_t *shm_ring_slot_payload_const(const ShmRing *ring, uint32_t slot_idx) {
    return ring->payload + ((size_t)slot_idx * (size_t)ring->slot_payload_size);
}

#ifdef __cplusplus
}
#endif

#endif
