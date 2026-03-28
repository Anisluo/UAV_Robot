#define _POSIX_C_SOURCE 200809L

#include "dev.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "can_socketcan.h"
#include "log.h"
#include "proto_robomodule_drv.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CAR_CAN_GROUP 0U
#define CAR_DEFAULT_LIMIT_PWM 800
#define CAR_MAX_RPM 4000
#define CAR_MM_PER_SEC_PER_RPM 20.0
#define CAR_TRACK_WIDTH_MM 600.0
#define CAR_INTER_FRAME_DELAY_MS 4L
#define CAR_STAGE_DELAY_MS 150L

static const uint8_t k_left_motors[] = {1U, 5U, 2U};
static const uint8_t k_right_motors[] = {4U, 6U, 3U};

static bool g_car_ready = false;
static bool g_last_command_valid = false;
static int g_last_left_rpm = 0;
static int g_last_right_rpm = 0;

static void car_sleep_ms(long ms) {
    struct timespec ts;

    if (ms <= 0) {
        return;
    }

    ts.tv_sec = ms / 1000L;
    ts.tv_nsec = (ms % 1000L) * 1000000L;
    while (nanosleep(&ts, &ts) != 0) {
    }
}

static int car_clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int car_round_to_int(double value) {
    if (value >= 0.0) {
        return (int)(value + 0.5);
    }
    return (int)(value - 0.5);
}

static bool car_send_frame(const RoboModuleDrvFrame *frame) {
    if (frame == NULL) {
        return false;
    }
    return can_socketcan_send_std(frame->can_id, frame->data);
}

static bool car_send_reset(uint8_t motor_num) {
    RoboModuleDrvFrame frame;

    if (!proto_robomodule_drv_encode_reset(CAR_CAN_GROUP, motor_num, &frame)) {
        return false;
    }
    return car_send_frame(&frame);
}

static bool car_send_mode(uint8_t motor_num, uint8_t mode) {
    RoboModuleDrvFrame frame;

    if (!proto_robomodule_drv_encode_mode(CAR_CAN_GROUP, motor_num, mode, &frame)) {
        return false;
    }
    return car_send_frame(&frame);
}

static bool car_send_velocity(uint8_t motor_num, int rpm, int limit_pwm) {
    RoboModuleDrvFrame frame;

    if (!proto_robomodule_drv_encode_velocity(CAR_CAN_GROUP,
                                              motor_num,
                                              limit_pwm,
                                              rpm,
                                              &frame)) {
        return false;
    }
    return car_send_frame(&frame);
}

static bool car_send_motor_group(const uint8_t *motors,
                                 size_t motor_count,
                                 int rpm,
                                 int direction_sign,
                                 int limit_pwm) {
    for (size_t i = 0; i < motor_count; ++i) {
        if (!car_send_velocity(motors[i], rpm * direction_sign, limit_pwm)) {
            return false;
        }
        car_sleep_ms(CAR_INTER_FRAME_DELAY_MS);
    }
    return true;
}

static bool car_set_lr_rpm(int left_rpm, int right_rpm, int limit_pwm) {
    if (!car_send_motor_group(k_left_motors,
                              sizeof(k_left_motors) / sizeof(k_left_motors[0]),
                              left_rpm,
                              +1,
                              limit_pwm)) {
        return false;
    }
    if (!car_send_motor_group(k_right_motors,
                              sizeof(k_right_motors) / sizeof(k_right_motors[0]),
                              right_rpm,
                              -1,
                              limit_pwm)) {
        return false;
    }

    g_last_left_rpm = left_rpm;
    g_last_right_rpm = right_rpm;
    g_last_command_valid = true;
    return true;
}

static bool car_run_init_sequence(void) {
    for (uint8_t motor_num = 1; motor_num <= 6U; ++motor_num) {
        if (!car_send_reset(motor_num)) {
            return false;
        }
        car_sleep_ms(CAR_INTER_FRAME_DELAY_MS);
    }

    car_sleep_ms(CAR_STAGE_DELAY_MS);

    for (uint8_t motor_num = 1; motor_num <= 6U; ++motor_num) {
        if (!car_send_mode(motor_num, ROBOMODULE_DRV_MODE_VELOCITY_CLOSED_LOOP)) {
            return false;
        }
        car_sleep_ms(CAR_INTER_FRAME_DELAY_MS);
    }

    car_sleep_ms(CAR_STAGE_DELAY_MS);
    g_last_command_valid = false;
    g_last_left_rpm = 0;
    g_last_right_rpm = 0;
    return true;
}

static bool car_ensure_ready(void) {
    if (g_car_ready) {
        return true;
    }

    if (can_socketcan_init() != 0) {
        return false;
    }
    if (!car_run_init_sequence()) {
        log_error("core.dev.car", "chassis init sequence failed");
        return false;
    }

    g_car_ready = true;
    log_info("core.dev.car", "chassis ready in RoboModule velocity mode");
    return true;
}

bool car_set_velocity(int linear_mm_s, int angular_mdeg_s) {
    double omega_rad_s;
    double yaw_component_mm_s;
    double left_mm_s;
    double right_mm_s;
    int left_rpm;
    int right_rpm;

    if (!car_ensure_ready()) {
        return false;
    }

    omega_rad_s = ((double)angular_mdeg_s) * M_PI / 180000.0;
    yaw_component_mm_s = omega_rad_s * (CAR_TRACK_WIDTH_MM / 2.0);

    left_mm_s = (double)linear_mm_s - yaw_component_mm_s;
    right_mm_s = (double)linear_mm_s + yaw_component_mm_s;

    left_rpm = car_clamp_int(car_round_to_int(left_mm_s / CAR_MM_PER_SEC_PER_RPM),
                             -CAR_MAX_RPM,
                             CAR_MAX_RPM);
    right_rpm = car_clamp_int(car_round_to_int(right_mm_s / CAR_MM_PER_SEC_PER_RPM),
                              -CAR_MAX_RPM,
                              CAR_MAX_RPM);

    if (g_last_command_valid &&
        left_rpm == g_last_left_rpm &&
        right_rpm == g_last_right_rpm) {
        return true;
    }

    if (!car_set_lr_rpm(left_rpm, right_rpm, CAR_DEFAULT_LIMIT_PWM)) {
        return false;
    }

    log_info("core.dev.car",
             "set_velocity linear=%dmm/s angular=%dmdeg/s -> left=%drpm right=%drpm",
             linear_mm_s,
             angular_mdeg_s,
             left_rpm,
             right_rpm);
    return true;
}
