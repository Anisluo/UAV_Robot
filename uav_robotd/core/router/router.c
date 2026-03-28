#include <string.h>
#include "router.h"
#include "proto.h"
#include "log.h"

void router_handle(const Event *ev, EventBus *bus) {
    if (ev->type != EVT_IO_CMD_RAW) {
        return;
    }

    CommandType cmd = proto_parse_command(ev->data.io.text);
    if (cmd == CMD_INVALID) {
        log_warn("router", "drop invalid raw command: %s", ev->data.io.text);
        return;
    }

    Event out = {0};
    out.type = EVT_CMD_REQUEST;
    out.ts_ms = ev->ts_ms;
    out.data.command.cmd = cmd;
    event_bus_publish(bus, &out);
    log_info("router", "route raw command '%s' -> %s", ev->data.io.text, proto_command_name(cmd));
}
