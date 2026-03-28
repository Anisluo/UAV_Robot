CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Icommon/include -O2
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
	uav_robotd/core/channel/can_channel.c \
	uav_robotd/core/channel/mesh_channel.c \
	uav_robotd/core/channel/bc_channel.c \
	uav_robotd/core/channel/uart_channel.c \
	uav_robotd/core/proto/proto_zdt_arm.c \
	uav_robotd/core/proto/proto_robomodule_drv.c \
	uav_robotd/core/proto/proto_gripper_uart.c \
	uav_robotd/core/proto/proto_platform_lock.c \
	uav_robotd/core/proto/proto_mesh_link.c \
	uav_robotd/core/dev/arm.c \
	uav_robotd/core/dev/car.c \
	uav_robotd/core/dev/platform.c \
	uav_robotd/core/dev/relay.c \
	uav_robotd/core/dev/gripper.c \
	uav_robotd/app/tasks/task_battery_pick.c \
	uav_robotd/drv/dev/can_socketcan.c \
	uav_robotd/drv/dev/uart_posix.c \
	uav_robotd/drv/dev/mesh_eth.c \
	uav_robotd/drv/io/gpio_sysfs.c

CLI_SRCS := tools/uav_cli.c

ROBOTD_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(ROBOTD_SRCS))
CLI_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(CLI_SRCS))

.PHONY: all clean run install test dirs proc_realsense proc_npu proc_car proc_gripper proc_arm proc_airport

all: dirs $(BIN_DIR)/uav_robotd $(BIN_DIR)/uav_cli proc_realsense proc_npu proc_car proc_gripper proc_arm proc_airport

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
		$(OBJ_DIR)/uav_robotd/drv/io \
		$(OBJ_DIR)/tools

$(BIN_DIR)/uav_robotd: dirs $(ROBOTD_OBJS)
	$(CC) $(ROBOTD_OBJS) -o $@ $(LDFLAGS)

$(BIN_DIR)/uav_cli: dirs $(CLI_OBJS)
	$(CC) $(CLI_OBJS) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

run: all
	$(BIN_DIR)/uav_robotd

install: all
	@mkdir -p /usr/local/bin
	cp $(BIN_DIR)/uav_robotd /usr/local/bin/uav_robotd
	cp $(BIN_DIR)/uav_cli /usr/local/bin/uav_cli

test: all
	$(BIN_DIR)/uav_robotd --self-test
	$(BIN_DIR)/uav_cli start
	$(BIN_DIR)/uav_cli estop
	$(BIN_DIR)/uav_cli reset
	$(BIN_DIR)/uav_cli shutdown

proc_realsense:
	$(MAKE) -C proc_realsense

proc_npu:
	$(MAKE) -C proc_npu

proc_car:
	$(MAKE) -C proc_car

proc_gripper:
	$(MAKE) -C proc_gripper

proc_arm:
	$(MAKE) -C proc_arm

proc_airport:
	$(MAKE) -C proc_airport

clean:
	rm -rf $(BUILD_DIR)
	$(MAKE) -C proc_realsense clean
	$(MAKE) -C proc_npu clean
	$(MAKE) -C proc_car clean
	$(MAKE) -C proc_gripper clean
	$(MAKE) -C proc_arm clean
	$(MAKE) -C proc_airport clean
