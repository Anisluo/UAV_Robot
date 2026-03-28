#ifndef UAV_UART_POSIX_H
#define UAV_UART_POSIX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int uart_posix_init(void);
int uart_posix_open(const char *path, int baudrate);
void uart_posix_close(int fd);
bool uart_posix_write_all(int fd, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
