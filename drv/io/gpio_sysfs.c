#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "event.h"
#include "io.h"

void io_init(IOAdapter *io, const char **script, size_t script_len) {
    io->script = script;
    io->script_len = script_len;
    io->next_idx = 0;
}

bool io_read_command(IOAdapter *io, char *out, size_t out_len, int timeout_ms) {
    if (out == NULL || out_len == 0) {
        return false;
    }

    if (io->script != NULL && io->next_idx < io->script_len) {
        const char *src = io->script[io->next_idx];
        const char *nul = memchr(src, '\0', out_len - 1);
        size_t n = (nul != NULL) ? (size_t)(nul - src) : (out_len - 1);
        memcpy(out, src, n);
        out[n] = '\0';
        io->next_idx++;
        return true;
    }

    struct pollfd pfd;
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    pfd.revents = 0;
    if (poll(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN)) {
        return false;
    }

    if (fgets(out, (int)out_len, stdin) == NULL) {
        return false;
    }

    out[strcspn(out, "\r\n")] = '\0';
    return out[0] != '\0';
}
