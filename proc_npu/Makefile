CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O2 -Iinclude -Isrc -I../common/include -I../common/include/abi
LDFLAGS ?=
LDLIBS ?= -lrknnrt -lpthread -lrt

BUILD_DIR := build
OBJ_DIR := $(BUILD_DIR)/obj
BIN_DIR := $(BUILD_DIR)/bin
TARGET := $(BIN_DIR)/proc_npu

SRCS := \
	app/npu_application.cpp \
	core/npu_pipeline.cpp \
	src/main.cpp \
	src/shm_reader.cpp \
	src/preprocess.cpp \
	src/npu_infer.cpp \
	src/postprocess.cpp \
	src/ctrl_server.cpp \
	src/health.cpp \
	src/result_publisher.cpp

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
