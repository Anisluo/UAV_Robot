CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -I../common/include
LDFLAGS  ?=
LDLIBS   ?= -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_gripper

SRCS := \
	src/main.cpp \
	../uav_robotd/drv/dev/uart_posix.c \
	../uav_robotd/core/proto/proto_gripper_uart.c \
	../uav_robotd/core/log/log.c

OBJS := \
	$(OBJ_DIR)/main.o \
	$(OBJ_DIR)/uart_posix.o \
	$(OBJ_DIR)/proto_gripper_uart.o \
	$(OBJ_DIR)/log.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/main.o: src/main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/uart_posix.o: ../uav_robotd/drv/dev/uart_posix.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_gripper_uart.o: ../uav_robotd/core/proto/proto_gripper_uart.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
