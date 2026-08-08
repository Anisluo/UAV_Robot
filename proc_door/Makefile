CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude -I../common/include
LDFLAGS  ?=
LDLIBS   ?= -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_door

SRCS := \
	app/door_application.cpp \
	core/door_runtime.cpp \
	core/modbus_rtu.cpp \
	src/main.cpp \
	../uav_robotd/core/log/log.c

OBJS := \
	$(OBJ_DIR)/app/door_application.o \
	$(OBJ_DIR)/core/door_runtime.o \
	$(OBJ_DIR)/core/modbus_rtu.o \
	$(OBJ_DIR)/src/main.o \
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

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
