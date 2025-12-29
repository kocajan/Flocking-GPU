# =========================
# Compiler / flags
# =========================
CXX := g++
CC  := gcc

CXXFLAGS := -std=c++20 -O2 -Wall -Wextra \
            -Iinclude \
            -Ithird_party \
            -Ithird_party/glad/include

CFLAGS := -O2 -Wall -Wextra \
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

# =========================
# Objects
# =========================
CPP_OBJ := $(CPP_SRC:%.cpp=$(BUILD_DIR)/%.o)
C_OBJ   := $(C_SRC:%.c=$(BUILD_DIR)/%.o)
OBJ     := $(CPP_OBJ) $(C_OBJ)

# =========================
# Targets
# =========================
all: $(BIN)

$(BIN): $(OBJ)
	$(CXX) $^ -o $@ $(LIBS)

# =========================
# Compile rules
# =========================
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# =========================
# Utilities
# =========================

# clean = remove build artifacts, keep third party sources
clean:
	rm -rf $(BUILD_DIR) $(BIN)

# fclean = full clean including any extra generated dirs
fclean: clean
	# remove optional caches or generated vendor outputs here
	# (keeps actual third_party source tree untouched)
	@echo "Full cleanup done."

# rebuild convenience target
re: fclean all

run: all
	./$(BIN)

.PHONY: all clean fclean re run

