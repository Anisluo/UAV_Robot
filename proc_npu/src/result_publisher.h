#ifndef UAV_PROC_NPU_RESULT_PUBLISHER_H
#define UAV_PROC_NPU_RESULT_PUBLISHER_H

#include "abi/msg_types.h"

// Sends UavCResult binary datagrams to known receivers (proc_gateway, app).
// Uses SOCK_DGRAM sendto(); silently drops if the destination socket does not exist.
class ResultPublisher {
public:
    ResultPublisher();
    ~ResultPublisher();

    bool open();
    void close();

    // Publish result to all registered destination paths.
    void publish(const UavCResult &result);

private:
    int fd_{-1};

    bool send_to(const char *path, const UavCResult &result);
};

#endif // UAV_PROC_NPU_RESULT_PUBLISHER_H
