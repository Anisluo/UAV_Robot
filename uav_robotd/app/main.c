#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "event_bus.h"
#include "can_channel.h"
#include "reactor.h"
#include "router.h"
#include "scheduler.h"
#include "supervisor.h"
#include "system.h"
#include "log.h"
#include "proto.h"

static void print_help(void) {
    puts("uav_robotd commands from stdin:");
    puts("  START_BATTERY_PICK | START");
    puts("  ESTOP");
    puts("  RESET");
    puts("  SHUTDOWN");
    puts("Options:");
    puts("  --listen-port <port>   listen for UDP text commands on the given port");
    puts("  --self-test            run built-in test script");
}

static bool parse_port(const char *text, unsigned short *out_port) {
    char *end = NULL;
    unsigned long value;

    if (text == NULL || out_port == NULL) {
        return false;
    }

    value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || value == 0 || value > 65535UL) {
        return false;
    }

    *out_port = (unsigned short)value;
    return true;
}

int main(int argc, char **argv) {
    bool self_test = false;
    unsigned short listen_port = 0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--self-test") == 0) {
            self_test = true;
            continue;
        }
        if (strcmp(argv[i], "--listen-port") == 0) {
            if (i + 1 >= argc || !parse_port(argv[i + 1], &listen_port)) {
                fprintf(stderr, "invalid listen port\n");
                print_help();
                return 1;
            }
            ++i;
            continue;
        }

        fprintf(stderr, "unknown argument: %s\n", argv[i]);
        print_help();
        return 1;
    }

    const char *script[] = {"START", "ESTOP", "RESET", "START", "SHUTDOWN"};

    EventBus bus;
    Reactor reactor;
    SystemState state;
    Scheduler scheduler;

    event_bus_init(&bus);
    system_state_init(&state);
    scheduler_init(&scheduler, &bus, &state);
    if (!reactor_init(&reactor,
                      self_test ? script : NULL,
                      self_test ? (sizeof(script) / sizeof(script[0])) : 0,
                      listen_port)) {
        fprintf(stderr, "failed to initialize command input\n");
        return 1;
    }

    log_info("main", "uav_robotd started");
    if (!self_test) {
        print_help();
        if (listen_port != 0) {
            log_info("main", "listening for UDP commands on port %u", (unsigned int)listen_port);
        }
    }

    int safety_loops = 0;
    while (!state.shutdown) {
        reactor_poll(&reactor, &bus, 200);
        can_channel_poll(&bus);

        Event ev;
        while (event_bus_try_pop(&bus, &ev)) {
            router_handle(&ev, &bus);
            supervisor_handle(&ev, &bus, &state);
            scheduler_handle(&ev, &bus, &state, &scheduler);

            if (ev.type == EVT_CMD_ACK) {
                log_info("ack", "ACK %s: %s", proto_command_name(ev.data.feedback.cmd), ev.data.feedback.reason);
            } else if (ev.type == EVT_CMD_NACK) {
                log_warn("ack", "NACK %s: %s", proto_command_name(ev.data.feedback.cmd), ev.data.feedback.reason);
            } else if (ev.type == EVT_DEVICE_FEEDBACK) {
                log_info("can",
                         "%s addr=%u cmd=0x%02X status=0x%02X %s",
                         ev.data.device.source,
                         (unsigned int)ev.data.device.addr,
                         (unsigned int)ev.data.device.cmd,
                         (unsigned int)ev.data.device.status,
                         ev.data.device.text);
            }
        }

        if (self_test && ++safety_loops > 200) {
            log_warn("main", "self-test timeout, forcing shutdown");
            break;
        }
    }

    reactor_close(&reactor);
    log_info("main", "uav_robotd stopped");
    return state.fault ? 1 : 0;
}
