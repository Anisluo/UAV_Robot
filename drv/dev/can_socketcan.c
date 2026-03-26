#define _DEFAULT_SOURCE

#include "can_socketcan.h"

#include <errno.h>
#include <linux/can.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "log.h"

#define CAN_DEFAULT_IFACE "can3"

static int g_can_fd = -1;
static char g_can_iface[IF_NAMESIZE] = CAN_DEFAULT_IFACE;

int can_socketcan_init(void) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    const char *iface = getenv("UAV_CAN_IFACE");
    int fd;

    if (g_can_fd >= 0) {
        return 0;
    }

    if (iface != NULL && iface[0] != '\0') {
        (void)snprintf(g_can_iface, sizeof(g_can_iface), "%s", iface);
    }

    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        log_error("drv.dev.can", "socket(PF_CAN) failed: %s", strerror(errno));
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    (void)snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", g_can_iface);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        log_error("drv.dev.can",
                  "SIOCGIFINDEX %s failed: %s",
                  g_can_iface,
                  strerror(errno));
        close(fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("drv.dev.can",
                  "bind(%s) failed: %s",
                  g_can_iface,
                  strerror(errno));
        close(fd);
        return -1;
    }

    g_can_fd = fd;
    log_info("drv.dev.can", "socketcan ready on %s", g_can_iface);
    return 0;
}

void can_socketcan_close(void) {
    if (g_can_fd >= 0) {
        close(g_can_fd);
        g_can_fd = -1;
    }
}

bool can_socketcan_send(uint32_t can_id,
                        bool is_extended_id,
                        const uint8_t *data,
                        size_t len) {
    struct can_frame frame;
    ssize_t written;

    if (data == NULL) {
        log_error("drv.dev.can", "send got null data");
        return false;
    }
    if (len > sizeof(frame.data)) {
        log_error("drv.dev.can", "send len=%u exceeds 8", (unsigned int)len);
        return false;
    }

    if (can_socketcan_init() != 0) {
        return false;
    }

    memset(&frame, 0, sizeof(frame));
    if (is_extended_id) {
        frame.can_id = (canid_t)(can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    } else {
        frame.can_id = (canid_t)(can_id & CAN_SFF_MASK);
    }
    frame.can_dlc = (uint8_t)len;
    memcpy(frame.data, data, len);

    written = write(g_can_fd, &frame, sizeof(frame));
    if (written != (ssize_t)sizeof(frame)) {
        log_error("drv.dev.can",
                  "write(can_id=0x%08X, ext=%s) failed: %s",
                  (unsigned int)can_id,
                  is_extended_id ? "true" : "false",
                  strerror(errno));
        return false;
    }

    return true;
}

bool can_socketcan_receive(CanSocketFrame *out_frame) {
    struct can_frame frame;
    ssize_t nread;

    if (out_frame == NULL) {
        return false;
    }
    if (can_socketcan_init() != 0) {
        return false;
    }

    nread = recv(g_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
    if (nread < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;
        }
        log_error("drv.dev.can", "recv failed: %s", strerror(errno));
        return false;
    }
    if (nread != (ssize_t)sizeof(frame)) {
        log_warn("drv.dev.can", "short can frame read: %d", (int)nread);
        return false;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
    out_frame->can_id = out_frame->is_extended_id
                            ? (uint32_t)(frame.can_id & CAN_EFF_MASK)
                            : (uint32_t)(frame.can_id & CAN_SFF_MASK);
    out_frame->len = frame.can_dlc;
    memcpy(out_frame->data, frame.data, frame.can_dlc);
    return true;
}

bool can_socketcan_send_std(uint16_t can_id, const uint8_t data[8]) {
    return can_socketcan_send(can_id, false, data, 8);
}

bool can_socketcan_send_ext(uint32_t can_id, const uint8_t *data, size_t len) {
    return can_socketcan_send(can_id, true, data, len);
}
