#include <string.h>
#include <time.h>
#include <poll.h>
#include <unistd.h>
#include <stdio.h>
#include "reactor.h"

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

void reactor_init(Reactor *reactor, const char **script, size_t script_len) {
    reactor->script = script;
    reactor->script_len = script_len;
    reactor->next_idx = 0;
}

void reactor_poll(Reactor *reactor, EventBus *bus, int timeout_ms) {
    Event ev;
    if (reactor->script && reactor->next_idx < reactor->script_len) {
        memset(&ev, 0, sizeof(ev));
        ev.type = EVT_IO_CMD_RAW;
        ev.ts_ms = now_ms();
        strncpy(ev.data.io.text, reactor->script[reactor->next_idx], UAV_TEXT_MAX - 1);
        reactor->next_idx++;
        event_bus_publish(bus, &ev);
    } else {
        struct pollfd pfd;
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        if (poll(&pfd, 1, timeout_ms) > 0 && (pfd.revents & POLLIN)) {
            char line[UAV_TEXT_MAX] = {0};
            if (fgets(line, sizeof(line), stdin) != NULL) {
                line[strcspn(line, "\r\n")] = '\0';
                if (line[0] != '\0') {
                    memset(&ev, 0, sizeof(ev));
                    ev.type = EVT_IO_CMD_RAW;
                    ev.ts_ms = now_ms();
                    strncpy(ev.data.io.text, line, UAV_TEXT_MAX - 1);
                    event_bus_publish(bus, &ev);
                }
            }
        }
    }

    memset(&ev, 0, sizeof(ev));
    ev.type = EVT_TIMER_TICK;
    ev.ts_ms = now_ms();
    event_bus_publish(bus, &ev);
}
