# =========================
# Compiler / flags
# =========================
CXX := g++
CC  := gcc
NVCC := nvcc

CXXFLAGS := -std=c++20 -O2 -Wall -Wextra \
            -Iinclude \
            -Ithird_party \
            -Ithird_party/glad/include

CFLAGS := -O2 -Wall -Wextra \
          -Ithird_party/glad/include

NVCCFLAGS := -std=c++20 -O2 \
             -Iinclude \
             -Ithird_party \
             -Ithird_party/glad/include

# =========================
# Libraries
# =========================
LIBS := -lglfw -lGL -ldl -lpthread

# =========================
# Directories
# =========================
SRC_DIR   := src
TP_DIR    := third_party
BUILD_DIR := build
BIN       := flocking

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

# =========================
# Targets
# =========================
all: $(BIN)

$(BIN): $(OBJ)
	$(NVCC) $^ -o $@ $(LIBS) -lcudart

# =========================
# Run options
# =========================
CFG ?= cfg

run: all
	./$(BIN) run $(CFG)

app: run

lab: all
	./$(BIN) experiment $(CFG)

# =========================
# Compile rules
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
# Utilities
# =========================
clean:
	rm -rf $(BUILD_DIR) $(BIN)

fclean: clean
	@echo "Full cleanup done."

re: fclean all

.PHONY: all clean fclean re run app lab
