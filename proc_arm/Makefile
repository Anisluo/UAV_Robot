CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude -I../common/include
LDFLAGS  ?=
LDLIBS   ?= -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_arm

SRCS := \
	app/arm_application.cpp \
	core/arm_controller.cpp \
	core/arm_runtime.cpp \
	src/main.cpp \
	../uav_robotd/core/dev/arm.c \
	../uav_robotd/core/dev/gripper.c \
	../uav_robotd/core/dev/relay.c \
	../uav_robotd/core/proto/proto_gripper_uart.c \
	../uav_robotd/core/proto/proto_zdt_arm.c \
	../uav_robotd/core/log/log.c \
	../uav_robotd/drv/dev/uart_posix.c

OBJS := \
	$(OBJ_DIR)/app/arm_application.o \
	$(OBJ_DIR)/core/arm_controller.o \
	$(OBJ_DIR)/core/arm_runtime.o \
	$(OBJ_DIR)/src/main.o \
	$(OBJ_DIR)/arm.o \
	$(OBJ_DIR)/gripper.o \
	$(OBJ_DIR)/relay.o \
	$(OBJ_DIR)/proto_gripper_uart.o \
	$(OBJ_DIR)/proto_zdt_arm.o \
	$(OBJ_DIR)/log.o \
	$(OBJ_DIR)/uart_posix.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/arm.o: ../uav_robotd/core/dev/arm.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/gripper.o: ../uav_robotd/core/dev/gripper.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/relay.o: ../uav_robotd/core/dev/relay.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_gripper_uart.o: ../uav_robotd/core/proto/proto_gripper_uart.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_zdt_arm.o: ../uav_robotd/core/proto/proto_zdt_arm.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/uart_posix.o: ../uav_robotd/drv/dev/uart_posix.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
