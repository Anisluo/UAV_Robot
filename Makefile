CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Iinclude -O2
LDFLAGS ?=

BUILD_DIR := build
BIN_DIR := $(BUILD_DIR)/bin
OBJ_DIR := $(BUILD_DIR)/obj

ROBOTD_SRCS := \
	core/main.c \
	core/bus/log.c \
	core/bus/event_bus.c \
	core/reactor/reactor.c \
	core/router/router.c \
	core/command/command_dispatcher.c \
	core/task/system_state.c \
	core/task/device_registry.c \
	core/task/task_manager.c \
	app/task_battery_pick.c \
	drv/proto/proto.c \
	drv/dev/dev.c

CLI_SRCS := tools/uav_cli.c

ROBOTD_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(ROBOTD_SRCS))
CLI_OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(CLI_SRCS))

.PHONY: all clean run install test dirs

all: dirs $(BIN_DIR)/uav_robotd $(BIN_DIR)/uav_cli

dirs:
	@mkdir -p $(BIN_DIR) $(OBJ_DIR)/core/bus $(OBJ_DIR)/core/reactor $(OBJ_DIR)/core/router $(OBJ_DIR)/core/command $(OBJ_DIR)/core/task $(OBJ_DIR)/app $(OBJ_DIR)/drv/proto $(OBJ_DIR)/drv/dev $(OBJ_DIR)/tools

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

clean:
	rm -rf $(BUILD_DIR)
