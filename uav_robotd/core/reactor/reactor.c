#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "event.h"
#include "io.h"
#include "reactor.h"

static uint64_t now_ms(void) {
    struct timespec ts;
    if (timespec_get(&ts, TIME_UTC) == TIME_UTC) {
        return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
    }
    return (uint64_t)time(NULL) * 1000ULL;
}

bool reactor_init(Reactor *reactor, const char **script, size_t script_len, unsigned short listen_port) {
    return io_init(&reactor->io, script, script_len, listen_port);
}

void reactor_close(Reactor *reactor) {
    io_close(&reactor->io);
}

void reactor_poll(Reactor *reactor, EventBus *bus, int timeout_ms) {
    Event ev;
    char line[UAV_TEXT_MAX] = {0};
    if (io_read_command(&reactor->io, line, sizeof(line), timeout_ms)) {
        memset(&ev, 0, sizeof(ev));
        ev.type = EVT_IO_CMD_RAW;
        ev.ts_ms = now_ms();
        const char *nul = memchr(line, '\0', UAV_TEXT_MAX - 1);
        size_t n = (nul != NULL) ? (size_t)(nul - line) : (UAV_TEXT_MAX - 1);
        memcpy(ev.data.io.text, line, n);
        ev.data.io.text[n] = '\0';
        event_bus_publish(bus, &ev);
    }

    memset(&ev, 0, sizeof(ev));
    ev.type = EVT_TIMER_TICK;
    ev.ts_ms = now_ms();
    event_bus_publish(bus, &ev);
}
