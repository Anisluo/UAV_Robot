#ifndef UAV_PROC_GRASP_RESULT_LISTENER_H
#define UAV_PROC_GRASP_RESULT_LISTENER_H

#include <cstdint>

#include "abi/msg_types.h"

// Binds a Unix SOCK_DGRAM socket at UAV_NPU_RESULT_GRASP_PATH and drains
// any UavCResult datagrams pushed by proc_npu. Only the latest result is
// kept — grasp planning cares about the current world snapshot, not
// history.
class ResultListener {
public:
    ResultListener();
    ~ResultListener();

    bool open();
    void close();

    // Non-blocking: pulls every pending datagram and keeps the most recent.
    // Returns the number of datagrams drained this call.
    int poll();

    // Copies the latest result into `out`. Returns false if no result has
    // ever arrived. `age_ms_out` (optional) is filled with the age (ms)
    // of that snapshot relative to now.
    bool latest(UavCResult *out, uint64_t *age_ms_out) const;

    // Resets the "has_result" flag — useful after a grasp completes so the
    // next trigger waits for a fresh detection.
    void clear();

    bool ok() const { return fd_ >= 0; }

private:
    int fd_;
    mutable UavCResult latest_{};
    bool has_result_;
    uint64_t last_rx_ms_;
};

#endif
