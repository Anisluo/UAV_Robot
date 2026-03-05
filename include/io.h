#ifndef UAV_IO_H
#define UAV_IO_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    const char **script;
    size_t script_len;
    size_t next_idx;
} IOAdapter;

void io_init(IOAdapter *io, const char **script, size_t script_len);
bool io_read_command(IOAdapter *io, char *out, size_t out_len, int timeout_ms);

#endif
