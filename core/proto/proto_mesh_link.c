#include <ctype.h>
#include <string.h>
#include "proto.h"

static void normalize(char *s) {
    for (int i = 0; s[i] != '\0'; ++i) {
        s[i] = (char)toupper((unsigned char)s[i]);
    }
}

CommandType proto_parse_command(const char *raw) {
    char buf[UAV_TEXT_MAX] = {0};
    const char *nul = memchr(raw, '\0', sizeof(buf) - 1);
    size_t n = (nul != NULL) ? (size_t)(nul - raw) : (sizeof(buf) - 1);
    memcpy(buf, raw, n);
    buf[n] = '\0';
    normalize(buf);

    if (strcmp(buf, "START_BATTERY_PICK") == 0 || strcmp(buf, "START") == 0) {
        return CMD_START_BATTERY_PICK;
    }
    if (strcmp(buf, "ESTOP") == 0 || strcmp(buf, "EMERGENCY_STOP") == 0) {
        return CMD_EMERGENCY_STOP;
    }
    if (strcmp(buf, "RESET") == 0 || strcmp(buf, "RESET_FAULT") == 0) {
        return CMD_RESET_FAULT;
    }
    if (strcmp(buf, "SHUTDOWN") == 0 || strcmp(buf, "QUIT") == 0) {
        return CMD_SHUTDOWN;
    }
    return CMD_INVALID;
}

const char *proto_command_name(CommandType cmd) {
    switch (cmd) {
        case CMD_START_BATTERY_PICK: return "START_BATTERY_PICK";
        case CMD_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case CMD_RESET_FAULT: return "RESET_FAULT";
        case CMD_SHUTDOWN: return "SHUTDOWN";
        default: return "INVALID";
    }
}
