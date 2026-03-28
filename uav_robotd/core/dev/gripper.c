#include "dev.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "log.h"
#include "proto_gripper_uart.h"
#include "uart_posix.h"

bool gripper_open(bool open) {
    static const char *k_default_uart_path = "/dev/ttyUSB0";
    static const int k_gripper_baudrate = 115200;
    uint8_t cmd[16];
    size_t cmd_len = 0U;
    const char *uart_path = getenv("UAV_GRIPPER_UART_PATH");
    int fd;
    bool ok;

    if (uart_path == NULL || uart_path[0] == '\0') {
        uart_path = k_default_uart_path;
    }

    if (!proto_gripper_uart_encode_open(open, cmd, sizeof(cmd), &cmd_len)) {
        log_error("core.dev.gripper", "encode gripper command failed");
        return false;
    }

    fd = uart_posix_open(uart_path, k_gripper_baudrate);
    if (fd < 0) {
        log_error("core.dev.gripper",
                  "open uart failed path=%s baud=%d",
                  uart_path,
                  k_gripper_baudrate);
        return false;
    }

    ok = uart_posix_write_all(fd, cmd, cmd_len);
    uart_posix_close(fd);

    if (!ok) {
        log_error("core.dev.gripper",
                  "send gripper command failed path=%s action=%s",
                  uart_path,
                  open ? "open" : "close");
        return false;
    }

    log_info("core.dev.gripper",
             "gripper action=%s path=%s baud=%d bytes=%u",
             open ? "open" : "close",
             uart_path,
             k_gripper_baudrate,
             (unsigned int)cmd_len);
    return true;
}
