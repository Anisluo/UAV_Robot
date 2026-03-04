#include <stdio.h>
#include <string.h>

static void usage(const char *name) {
    printf("Usage: %s <start|estop|reset|shutdown>\n", name);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "start") == 0) {
        puts("START_BATTERY_PICK");
        return 0;
    }
    if (strcmp(argv[1], "estop") == 0) {
        puts("ESTOP");
        return 0;
    }
    if (strcmp(argv[1], "reset") == 0) {
        puts("RESET");
        return 0;
    }
    if (strcmp(argv[1], "shutdown") == 0) {
        puts("SHUTDOWN");
        return 0;
    }

    usage(argv[0]);
    return 1;
}
