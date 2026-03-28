CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude -I../common/include
LDFLAGS  ?=
LDLIBS   ?= -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_airport

SRCS := \
	app/airport_application.cpp \
	core/airport_runtime.cpp \
	src/main.cpp \
	../uav_robotd/core/dev/platform.c \
	../uav_robotd/drv/dev/can_socketcan.c \
	../uav_robotd/core/proto/proto_platform_lock.c \
	../uav_robotd/core/log/log.c

OBJS := \
	$(OBJ_DIR)/app/airport_application.o \
	$(OBJ_DIR)/core/airport_runtime.o \
	$(OBJ_DIR)/src/main.o \
	$(OBJ_DIR)/platform.o \
	$(OBJ_DIR)/can_socketcan.o \
	$(OBJ_DIR)/proto_platform_lock.o \
	$(OBJ_DIR)/log.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/platform.o: ../uav_robotd/core/dev/platform.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/can_socketcan.o: ../uav_robotd/drv/dev/can_socketcan.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_platform_lock.o: ../uav_robotd/core/proto/proto_platform_lock.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
