#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev.h"

static void usage(const char *prog) {
    fprintf(stderr,
            "Usage:\n"
            "  %s home-joint <joint> [home-mode]\n"
            "  %s set-zero-params <joint>\n"
            "  %s move-joint <joint> <target-deg>\n"
            "  %s estop-joint <joint>\n"
            "  %s estop\n"
            "  %s stop\n"
            "  %s demo <joint> <target-deg>\n"
            "\n"
            "Examples:\n"
            "  UAV_ARM_CAN_IFACE=can0 %s home-joint 1 2\n"
            "  UAV_ARM_CAN_IFACE=can0 %s set-zero-params 1\n"
            "  UAV_ARM_CAN_IFACE=can0 %s move-joint 1 30\n"
            "  UAV_ARM_CAN_IFACE=can0 %s estop-joint 1\n"
            "  UAV_ARM_CAN_IFACE=can0 %s estop\n"
            "  UAV_ARM_CAN_IFACE=can0 UAV_ARM_WAIT_REACHED=1 %s demo 1 45\n"
            "\n"
            "Environment:\n"
            "  UAV_ARM_CAN_IFACE         SocketCAN iface, default comes from arm.c\n"
            "  UAV_ARM_HOME_MODE         Homing mode, default 2 for single-joint tests\n"
            "  UAV_ARM_COLLISION_ZERO_SPEED_RPM  Collision detect speed, default 300\n"
            "  UAV_ARM_COLLISION_ZERO_CURRENT_MA Collision detect current, default 800\n"
            "  UAV_ARM_COLLISION_ZERO_TIME_MS    Collision detect time, default 60\n"
            "  UAV_ARM_WAIT_REACHED      1 waits until target reached\n"
            "  UAV_ARM_ACK_TIMEOUT_MS    ACK timeout in ms\n"
            "  UAV_ARM_REACHED_TIMEOUT_MS target reached timeout in ms\n",
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog,
            prog);
}

static int parse_int_arg(const char *text, int *out) {
    char *end = NULL;
    long value;

    if (text == NULL || out == NULL) {
        return 0;
    }
    value = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return 0;
    }
    *out = (int)value;
    return 1;
}

static int parse_float_arg(const char *text, float *out) {
    char *end = NULL;
    float value;

    if (text == NULL || out == NULL) {
        return 0;
    }
    value = strtof(text, &end);
    if (end == text || *end != '\0') {
        return 0;
    }
    *out = value;
    return 1;
}

int main(int argc, char **argv) {
    const char *iface = getenv("UAV_ARM_CAN_IFACE");

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    printf("arm_motor_test: iface=%s wait_reached=%s\n",
           (iface != NULL && iface[0] != '\0') ? iface : "(default)",
           getenv("UAV_ARM_WAIT_REACHED") != NULL ? getenv("UAV_ARM_WAIT_REACHED") : "0");

    if (strcmp(argv[1], "home-joint") == 0) {
        int joint = 0;
        int home_mode = 2;
        char home_mode_text[16];

        if ((argc != 3 && argc != 4) || !parse_int_arg(argv[2], &joint)) {
            usage(argv[0]);
            return 1;
        }
        if (argc == 4 && !parse_int_arg(argv[3], &home_mode)) {
            usage(argv[0]);
            return 1;
        }
        (void)snprintf(home_mode_text, sizeof(home_mode_text), "%d", home_mode);
        if (setenv("UAV_ARM_HOME_MODE", home_mode_text, 1) != 0) {
            perror("setenv UAV_ARM_HOME_MODE");
            return 1;
        }
        if (!arm_home_joint(joint)) {
            fprintf(stderr, "home joint %d failed\n", joint);
            return 2;
        }
        printf("home joint %d ok (mode=%d)\n", joint, home_mode);
        return 0;
    }

    if (strcmp(argv[1], "move-joint") == 0) {
        int joint = 0;
        float target_deg = 0.0f;
        if (argc != 4 || !parse_int_arg(argv[2], &joint) || !parse_float_arg(argv[3], &target_deg)) {
            usage(argv[0]);
            return 1;
        }
        if (!arm_move_joint_deg(joint, target_deg)) {
            fprintf(stderr, "move joint %d -> %.2f deg failed\n", joint, (double)target_deg);
            return 2;
        }
        printf("move joint %d -> %.2f deg ok\n", joint, (double)target_deg);
        return 0;
    }

    if (strcmp(argv[1], "set-zero-params") == 0) {
        int joint = 0;
        if (argc != 3 || !parse_int_arg(argv[2], &joint)) {
            usage(argv[0]);
            return 1;
        }
        if (!arm_set_zero_params(joint)) {
            fprintf(stderr, "set zero params joint %d failed\n", joint);
            return 2;
        }
        printf("set zero params joint %d ok\n", joint);
        return 0;
    }

    if (strcmp(argv[1], "stop") == 0) {
        if (!arm_stop()) {
            fprintf(stderr, "stop failed\n");
            return 2;
        }
        printf("stop ok\n");
        return 0;
    }

    if (strcmp(argv[1], "estop-joint") == 0) {
        int joint = 0;
        bool stop_ok;
        if (argc != 3 || !parse_int_arg(argv[2], &joint)) {
            usage(argv[0]);
            return 1;
        }
        stop_ok = arm_stop_joint(joint);
        dev_emergency_stop_all();
        if (!stop_ok) {
            fprintf(stderr, "estop-joint %d failed, emergency hook fired\n", joint);
            return 2;
        }
        printf("estop-joint %d ok\n", joint);
        return 0;
    }

    if (strcmp(argv[1], "estop") == 0) {
        bool stop_ok = arm_stop();
        dev_emergency_stop_all();
        if (!stop_ok) {
            fprintf(stderr, "estop: arm stop failed, emergency hook fired\n");
            return 2;
        }
        printf("estop ok\n");
        return 0;
    }

    if (strcmp(argv[1], "demo") == 0) {
        int joint = 0;
        float target_deg = 0.0f;
        if (argc != 4 || !parse_int_arg(argv[2], &joint) || !parse_float_arg(argv[3], &target_deg)) {
            usage(argv[0]);
            return 1;
        }
        printf("demo: home joint %d\n", joint);
        if (!arm_home_joint(joint)) {
            fprintf(stderr, "demo home joint %d failed\n", joint);
            return 2;
        }
        printf("demo: move joint %d -> %.2f deg\n", joint, (double)target_deg);
        if (!arm_move_joint_deg(joint, target_deg)) {
            fprintf(stderr, "demo move joint %d failed\n", joint);
            return 2;
        }
        printf("demo complete\n");
        return 0;
    }

    usage(argv[0]);
    return 1;
}
