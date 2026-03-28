CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -I../common/include
LDFLAGS  ?=
LDLIBS   ?= -lrt -lpthread

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_car

SRCS := \
	src/main.cpp \
	../uav_robotd/core/dev/car.c \
	../uav_robotd/drv/dev/can_socketcan.c \
	../uav_robotd/core/proto/proto_robomodule_drv.c \
	../uav_robotd/core/log/log.c

OBJS := \
	$(OBJ_DIR)/main.o \
	$(OBJ_DIR)/car.o \
	$(OBJ_DIR)/can_socketcan.o \
	$(OBJ_DIR)/proto_robomodule_drv.o \
	$(OBJ_DIR)/log.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/main.o: src/main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/car.o: ../uav_robotd/core/dev/car.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/can_socketcan.o: ../uav_robotd/drv/dev/can_socketcan.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_robomodule_drv.o: ../uav_robotd/core/proto/proto_robomodule_drv.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
