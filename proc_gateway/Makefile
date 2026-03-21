CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -I../include -I../include/abi
LDFLAGS  ?=
LDLIBS   ?= -ljpeg -lrt

BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
BIN_DIR   := $(BUILD_DIR)/bin
TARGET    := $(BIN_DIR)/proc_gateway

SRCS := \
	src/main.cpp \
	src/shm_reader.cpp \
	src/jpeg_encoder.cpp \
	src/ctrl_client.cpp

OBJS := $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

.PHONY: all clean dirs install-deps

all: dirs $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR) $(BIN_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ_DIR)/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

install-deps:
	sudo apt-get install -y libjpeg-dev

clean:
	rm -rf $(BUILD_DIR)
