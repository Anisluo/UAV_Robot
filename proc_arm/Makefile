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
	core/arm_runtime.cpp \
	src/main.cpp \
	../uav_robotd/core/dev/arm.c \
	../uav_robotd/core/proto/proto_zdt_arm.c \
	../uav_robotd/core/log/log.c

OBJS := \
	$(OBJ_DIR)/app/arm_application.o \
	$(OBJ_DIR)/core/arm_runtime.o \
	$(OBJ_DIR)/src/main.o \
	$(OBJ_DIR)/arm.o \
	$(OBJ_DIR)/proto_zdt_arm.o \
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

$(OBJ_DIR)/arm.o: ../uav_robotd/core/dev/arm.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/proto_zdt_arm.o: ../uav_robotd/core/proto/proto_zdt_arm.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/log.o: ../uav_robotd/core/log/log.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
