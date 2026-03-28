#ifndef UAV_IO_H
#define UAV_IO_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    const char **script;
    size_t script_len;
    size_t next_idx;
    int listen_fd;
    unsigned short listen_port;
} IOAdapter;

bool io_init(IOAdapter *io, const char **script, size_t script_len, unsigned short listen_port);
void io_close(IOAdapter *io);
bool io_read_command(IOAdapter *io, char *out, size_t out_len, int timeout_ms);

#endif
