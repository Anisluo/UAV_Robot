CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Iinclude -O2
LDFLAGS ?= -lm

BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin
OBJ_DIR := $(BUILD_DIR)/obj

ROBOTD_SRCS := \
	app/main.c \
	app/npu_detect.c \
	core/bus/bus.c \
	core/log/log.c \
	core/reactor/reactor.c \
	core/router/router.c \
	core/scheduler/scheduler.c \
	core/supervisor/supervisor.c \
	core/channel/can_channel.c \
	core/channel/mesh_channel.c \
	core/channel/bc_channel.c \
	core/channel/uart_channel.c \
	core/proto/proto_zdt_arm.c \
	core/proto/proto_robomodule_drv.c \
	core/proto/proto_gripper_uart.c \
	core/proto/proto_platform_lock.c \
	core/proto/proto_mesh_link.c \
	core/dev/arm.c \
	core/dev/car.c \
	core/dev/platform.c \
	core/dev/relay.c \
	core/dev/gripper.c \
	app/tasks/task_battery_pick.c \
	drv/dev/can_socketcan.c \
	drv/dev/uart_posix.c \
	drv/dev/mesh_eth.c \
	drv/io/gpio_sysfs.c

CLI_SRCS := tools/uav_cli.c

ROBOTD_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(ROBOTD_SRCS))
CLI_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(CLI_SRCS))

.PHONY: all clean run install test dirs proc_realsense proc_npu

all: dirs $(BIN_DIR)/uav_robotd $(BIN_DIR)/uav_cli proc_realsense proc_npu

dirs:
	@mkdir -p $(BIN_DIR) \
		$(OBJ_DIR)/app/tasks \
		$(OBJ_DIR)/core/bus \
		$(OBJ_DIR)/core/log \
		$(OBJ_DIR)/core/reactor \
		$(OBJ_DIR)/core/router \
		$(OBJ_DIR)/core/scheduler \
		$(OBJ_DIR)/core/supervisor \
		$(OBJ_DIR)/core/channel \
		$(OBJ_DIR)/core/proto \
		$(OBJ_DIR)/core/dev \
		$(OBJ_DIR)/drv/dev \
		$(OBJ_DIR)/drv/io \
		$(OBJ_DIR)/tools

$(BIN_DIR)/uav_robotd: $(ROBOTD_OBJS)
	$(CC) $(ROBOTD_OBJS) -o $@ $(LDFLAGS)

$(BIN_DIR)/uav_cli: $(CLI_OBJS)
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

clean:
	rm -rf $(BUILD_DIR)
	$(MAKE) -C proc_realsense clean
	$(MAKE) -C proc_npu clean
