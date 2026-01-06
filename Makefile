# =========================
# Compiler / flags
# =========================
CXX     := g++
CC      := gcc
NVCC    := nvcc

CXXFLAGS := -std=c++20 -O2 -Wall -Wextra -MMD -MP \
            -Iinclude \
            -Ithird_party \
            -Ithird_party/glad/include \
            -Ithird_party/giflib

CFLAGS := -O2 -Wall -Wextra -MMD -MP \
          -Ithird_party/glad/include

NVCCFLAGS := -std=c++20 -O2 -MMD -MP \
             -Iinclude \
             -Ithird_party \
             -Ithird_party/glad/include \
             -Ithird_party/giflib

# =========================
# Libraries
# =========================
LIBS := -lglfw -lGL -ldl -lpthread -lcudart

# =========================
# Directories
# =========================
SRC_DIR   := src
TP_DIR    := third_party
BUILD_DIR := build
BIN       := bin/flocking   # <-- executable now goes to ./bin

# =========================
# Source discovery
# =========================
CPP_SRC := $(shell find $(SRC_DIR) $(TP_DIR)/imgui -name '*.cpp')
C_SRC   := $(shell find $(TP_DIR)/glad -name '*.c')
CU_SRC  := $(shell find $(SRC_DIR) -name '*.cu')

# =========================
# Objects
# =========================
CPP_OBJ := $(CPP_SRC:%.cpp=$(BUILD_DIR)/%.o)
C_OBJ   := $(C_SRC:%.c=$(BUILD_DIR)/%.o)
CU_OBJ  := $(CU_SRC:%.cu=$(BUILD_DIR)/%.o)

OBJ := $(CPP_OBJ) $(C_OBJ) $(CU_OBJ)
DEPS := $(OBJ:.o=.d)

# =========================
# Default target
# =========================
all: $(BIN)

$(BIN): $(OBJ)
	@mkdir -p $(dir $@)
	$(NVCC) $^ -o $@ $(LIBS)

# =========================
# Build rules
# =========================
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.cu
	@mkdir -p $(dir $@)
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

# =========================
# Run helpers
# =========================
CFG ?= cfg

run: all
	./$(BIN) run $(CFG)

lab: all
	./$(BIN) experiment $(CFG)

app: run

# =========================
# Utilities
# =========================
clean:
	rm -rf $(BUILD_DIR)

fclean: clean
	rm -f $(BIN)

re: fclean all

-include $(DEPS)

.PHONY: all clean fclean re run lab app
