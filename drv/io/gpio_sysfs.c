#include <arpa/inet.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "event.h"
#include "io.h"

static bool io_open_listener(IOAdapter *io, unsigned short listen_port) {
    int fd;
    struct sockaddr_in addr;
    int reuse = 1;

    if (listen_port == 0) {
        return true;
    }

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return false;
    }

    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("setsockopt");
        close(fd);
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return false;
    }

    io->listen_fd = fd;
    io->listen_port = listen_port;
    return true;
}

bool io_init(IOAdapter *io, const char **script, size_t script_len, unsigned short listen_port) {
    io->script = script;
    io->script_len = script_len;
    io->next_idx = 0;
    io->listen_fd = -1;
    io->listen_port = 0;
    return io_open_listener(io, listen_port);
}

void io_close(IOAdapter *io) {
    if (io->listen_fd >= 0) {
        close(io->listen_fd);
        io->listen_fd = -1;
    }
    io->listen_port = 0;
}

bool io_read_command(IOAdapter *io, char *out, size_t out_len, int timeout_ms) {
    struct pollfd pfds[2];
    nfds_t nfds = 0;

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

    if (io->listen_fd >= 0) {
        pfds[nfds].fd = io->listen_fd;
        pfds[nfds].events = POLLIN;
        pfds[nfds].revents = 0;
        nfds++;
    }

    pfds[nfds].fd = STDIN_FILENO;
    pfds[nfds].events = POLLIN;
    pfds[nfds].revents = 0;
    nfds++;

    if (poll(pfds, nfds, timeout_ms) <= 0) {
        return false;
    }

    if (io->listen_fd >= 0 && (pfds[0].revents & POLLIN)) {
        ssize_t n = recvfrom(io->listen_fd, out, out_len - 1, 0, NULL, NULL);
        if (n <= 0) {
            return false;
        }
        out[n] = '\0';
        out[strcspn(out, "\r\n")] = '\0';
        return out[0] != '\0';
    }

    if (pfds[nfds - 1].revents & POLLIN) {
        if (fgets(out, (int)out_len, stdin) == NULL) {
            return false;
        }

        out[strcspn(out, "\r\n")] = '\0';
        return out[0] != '\0';
    }

    return false;
}
