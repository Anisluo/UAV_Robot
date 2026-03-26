CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -I../include -I../include/abi
LDFLAGS  ?=
LDLIBS   ?= -ljpeg -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_gateway

SRCS := \
	src/main.cpp \
	src/shm_reader.cpp \
	src/jpeg_encoder.cpp \
	src/ctrl_client.cpp \
	../core/proto/proto_gripper_uart.c \
	../core/proto/proto_zdt_arm.c \
	../drv/dev/uart_posix.c \
	../core/log/log.c

OBJS := \
	$(OBJ_DIR)/main.o \
	$(OBJ_DIR)/shm_reader.o \
	$(OBJ_DIR)/jpeg_encoder.o \
	$(OBJ_DIR)/ctrl_client.o \
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

$(OBJ_DIR)/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_zdt_arm.o: ../core/proto/proto_zdt_arm.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_gripper_uart.o: ../core/proto/proto_gripper_uart.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/uart_posix.o: ../drv/dev/uart_posix.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

install-deps:
	sudo apt-get install -y libjpeg-dev

clean:
	rm -rf $(BUILD_DIR)
