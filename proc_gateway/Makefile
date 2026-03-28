CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude -Isrc -I../common/include -I../common/include/abi
LDFLAGS  ?=
LDLIBS   ?= -ljpeg -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_gateway

SRCS := \
	app/gateway_application.cpp \
	core/gateway_runtime.cpp \
	src/main.cpp \
	src/shm_reader.cpp \
	src/jpeg_encoder.cpp \
	src/ctrl_client.cpp \
	../uav_robotd/core/dev/arm.c \
	../uav_robotd/core/proto/proto_gripper_uart.c \
	../uav_robotd/core/proto/proto_zdt_arm.c \
	../uav_robotd/drv/dev/uart_posix.c \
	../uav_robotd/core/log/log.c

OBJS := \
	$(OBJ_DIR)/app/gateway_application.o \
	$(OBJ_DIR)/core/gateway_runtime.o \
	$(OBJ_DIR)/src/main.o \
	$(OBJ_DIR)/src/shm_reader.o \
	$(OBJ_DIR)/src/jpeg_encoder.o \
	$(OBJ_DIR)/src/ctrl_client.o \
	$(OBJ_DIR)/arm.o \
	$(OBJ_DIR)/proto_gripper_uart.o \
	$(OBJ_DIR)/proto_zdt_arm.o \
	$(OBJ_DIR)/uart_posix.o \
	$(OBJ_DIR)/log.o

.PHONY: all clean dirs install-deps

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

$(OBJ_DIR)/proto_zdt_arm.o: ../uav_robotd/core/proto/proto_zdt_arm.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_gripper_uart.o: ../uav_robotd/core/proto/proto_gripper_uart.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/uart_posix.o: ../uav_robotd/drv/dev/uart_posix.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

install-deps:
	sudo apt-get install -y libjpeg-dev

clean:
	rm -rf $(BUILD_DIR)
