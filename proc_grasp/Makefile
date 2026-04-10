CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O2 -Iinclude -Isrc -I../common/include -I../common/include/abi
LDFLAGS ?=
LDLIBS ?= -lpthread -lrt

BUILD_DIR := build
OBJ_DIR := $(BUILD_DIR)/obj
BIN_DIR := $(BUILD_DIR)/bin
TARGET := $(BIN_DIR)/proc_grasp

SRCS := \
	app/grasp_application.cpp \
	core/grasp_pipeline.cpp \
	src/main.cpp \
	src/ctrl_server.cpp \
	src/result_listener.cpp \
	src/arm_client.cpp \
	src/hand_eye.cpp

OBJS := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(SRCS))

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
