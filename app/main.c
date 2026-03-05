#include <stdio.h>
#include <string.h>
#include "event_bus.h"
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
}

int main(int argc, char **argv) {
    bool self_test = false;
    if (argc > 1 && strcmp(argv[1], "--self-test") == 0) {
        self_test = true;
    }

    const char *script[] = {"START", "ESTOP", "RESET", "START", "SHUTDOWN"};

    EventBus bus;
    Reactor reactor;
    SystemState state;
    Scheduler scheduler;

    event_bus_init(&bus);
    system_state_init(&state);
    scheduler_init(&scheduler, &bus, &state);
    reactor_init(&reactor, self_test ? script : NULL, self_test ? (sizeof(script) / sizeof(script[0])) : 0);

    log_info("main", "uav_robotd started");
    if (!self_test) {
        print_help();
    }

    int safety_loops = 0;
    while (!state.shutdown) {
        reactor_poll(&reactor, &bus, 200);

        Event ev;
        while (event_bus_try_pop(&bus, &ev)) {
            router_handle(&ev, &bus);
            supervisor_handle(&ev, &bus, &state);
            scheduler_handle(&ev, &bus, &state, &scheduler);

            if (ev.type == EVT_CMD_ACK) {
                log_info("ack", "ACK %s: %s", proto_command_name(ev.data.feedback.cmd), ev.data.feedback.reason);
            } else if (ev.type == EVT_CMD_NACK) {
                log_warn("ack", "NACK %s: %s", proto_command_name(ev.data.feedback.cmd), ev.data.feedback.reason);
            }
        }

        if (self_test && ++safety_loops > 200) {
            log_warn("main", "self-test timeout, forcing shutdown");
            break;
        }
    }

    log_info("main", "uav_robotd stopped");
    return state.fault ? 1 : 0;
}
