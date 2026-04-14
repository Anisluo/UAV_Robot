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
	drv/arm_can.c \
	drv/proto_zdt_arm.c \
	drv/dev_stub.c \
	../uav_robotd/core/log/log.c

OBJS := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(filter %.cpp,$(SRCS))) \
        $(patsubst %.c,$(OBJ_DIR)/%.o,$(filter %.c,$(SRCS)))

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# log.c remains shared from uav_robotd
$(OBJ_DIR)/../uav_robotd/core/log/log.o: ../uav_robotd/core/log/log.c
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
