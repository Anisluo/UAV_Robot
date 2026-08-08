CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Icommon/include -Iinclude -O2
LDFLAGS ?= -lm

BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin
OBJ_DIR := $(BUILD_DIR)/obj

ROBOTD_SRCS := \
	uav_robotd/app/main.c \
	uav_robotd/app/npu_detect.c \
	uav_robotd/core/bus/bus.c \
	uav_robotd/core/log/log.c \
	uav_robotd/core/reactor/reactor.c \
	uav_robotd/core/router/router.c \
	uav_robotd/core/scheduler/scheduler.c \
	uav_robotd/core/supervisor/supervisor.c \
	uav_robotd/core/channel/mesh_channel.c \
	uav_robotd/core/channel/bc_channel.c \
	uav_robotd/core/channel/uart_channel.c \
	uav_robotd/core/proto/proto_mesh_link.c \
	uav_robotd/core/dev/arm_rpc_proxy.c \
	uav_robotd/core/dev/car_rpc_proxy.c \
	uav_robotd/core/dev/platform_rpc_proxy.c \
	uav_robotd/core/dev/gripper_rpc_proxy.c \
	uav_robotd/core/dev/relay.c \
	uav_robotd/app/tasks/task_battery_pick.c \
	uav_robotd/app/tasks/task_arm_demo.c \
	uav_robotd/app/tasks/task_pick_place.c \
	uav_robotd/app/tasks/task_face_track.c \
	uav_robotd/drv/dev/mesh_eth.c \
	uav_robotd/drv/io/gpio_sysfs.c

ARM_TEST_SRCS := \
	tools/arm_motor_test.c \
	uav_robotd/core/log/log.c \
	uav_robotd/core/proto/proto_zdt_arm.c \
	uav_robotd/core/dev/relay.c \
	uav_robotd/core/dev/arm.c

ROBOTD_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(ROBOTD_SRCS))
ARM_TEST_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(ARM_TEST_SRCS))

.PHONY: all clean run install test dirs proc_realsense proc_npu proc_gateway proc_car proc_gripper proc_arm proc_airport proc_grasp proc_door

all: dirs $(BIN_DIR)/uav_robotd $(BIN_DIR)/arm_motor_test proc_realsense proc_npu proc_gateway proc_car proc_gripper proc_arm proc_airport proc_grasp proc_door

dirs:
	@mkdir -p $(BIN_DIR) \
		$(OBJ_DIR)/uav_robotd/app/tasks \
		$(OBJ_DIR)/uav_robotd/core/bus \
		$(OBJ_DIR)/uav_robotd/core/log \
		$(OBJ_DIR)/uav_robotd/core/reactor \
		$(OBJ_DIR)/uav_robotd/core/router \
		$(OBJ_DIR)/uav_robotd/core/scheduler \
		$(OBJ_DIR)/uav_robotd/core/supervisor \
		$(OBJ_DIR)/uav_robotd/core/channel \
		$(OBJ_DIR)/uav_robotd/core/proto \
		$(OBJ_DIR)/uav_robotd/core/dev \
		$(OBJ_DIR)/uav_robotd/drv/dev \
		$(OBJ_DIR)/uav_robotd/drv/io

$(BIN_DIR)/uav_robotd: dirs $(ROBOTD_OBJS)
	$(CC) $(ROBOTD_OBJS) -o $@ $(LDFLAGS)

$(BIN_DIR)/arm_motor_test: dirs $(ARM_TEST_OBJS)
	$(CC) $(ARM_TEST_OBJS) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	$(BIN_DIR)/uav_robotd

install: all
	@mkdir -p /usr/local/bin
	cp $(BIN_DIR)/uav_robotd /usr/local/bin/uav_robotd

test: all
	$(BIN_DIR)/uav_robotd --self-test

proc_realsense:
	$(MAKE) -C proc_realsense

proc_npu:
	$(MAKE) -C proc_npu

proc_gateway:
	$(MAKE) -C proc_gateway

proc_car:
	$(MAKE) -C proc_car

proc_gripper:
	$(MAKE) -C proc_gripper

proc_arm:
	$(MAKE) -C proc_arm

proc_airport:
	$(MAKE) -C proc_airport

proc_grasp:
	$(MAKE) -C proc_grasp

proc_door:
	$(MAKE) -C proc_door

clean:
	rm -rf $(BUILD_DIR)
	$(MAKE) -C proc_realsense clean
	$(MAKE) -C proc_npu clean
	$(MAKE) -C proc_gateway clean
	$(MAKE) -C proc_car clean
	$(MAKE) -C proc_gripper clean
	$(MAKE) -C proc_arm clean
	$(MAKE) -C proc_airport clean
	$(MAKE) -C proc_grasp clean
	$(MAKE) -C proc_door clean
