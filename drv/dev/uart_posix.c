#define _DEFAULT_SOURCE

#include "uart_posix.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "log.h"

static speed_t uart_posix_baud_to_speed(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return (speed_t)0;
    }
}

int uart_posix_init(void) {
    return 0;
}

int uart_posix_open(const char *path, int baudrate) {
    struct termios tio;
    speed_t speed;
    int fd;

    if (path == NULL || path[0] == '\0') {
        log_error("drv.dev.uart", "uart path is empty");
        return -1;
    }

    speed = uart_posix_baud_to_speed(baudrate);
    if (speed == (speed_t)0) {
        log_error("drv.dev.uart", "unsupported baudrate=%d", baudrate);
        return -1;
    }

    fd = open(path, O_RDWR | O_NOCTTY | O_SYNC | O_CLOEXEC);
    if (fd < 0) {
        log_error("drv.dev.uart", "open(%s) failed: %s", path, strerror(errno));
        return -1;
    }

    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        log_error("drv.dev.uart", "cfset speed(%s) failed: %s", path, strerror(errno));
        close(fd);
        return -1;
    }

    if (tcflush(fd, TCIOFLUSH) != 0) {
        log_warn("drv.dev.uart", "tcflush(%s) failed: %s", path, strerror(errno));
    }
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        log_error("drv.dev.uart", "tcsetattr(%s) failed: %s", path, strerror(errno));
        close(fd);
        return -1;
    }

    log_info("drv.dev.uart", "uart ready path=%s baud=%d mode=8N1", path, baudrate);
    return fd;
}

void uart_posix_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

bool uart_posix_write_all(int fd, const uint8_t *data, size_t len) {
    size_t written_total = 0U;

    if (fd < 0 || data == NULL) {
        return false;
    }

    while (written_total < len) {
        ssize_t written = write(fd, data + written_total, len - written_total);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            log_error("drv.dev.uart", "write failed: %s", strerror(errno));
            return false;
        }
        if (written == 0) {
            log_error("drv.dev.uart", "write returned 0");
            return false;
        }
        written_total += (size_t)written;
    }

    if (tcdrain(fd) != 0) {
        log_error("drv.dev.uart", "tcdrain failed: %s", strerror(errno));
        return false;
    }

    return true;
}
