NVCC ?= nvcc
CXXFLAGS := -std=c++17

SRC = main.cu HashesKernels.cu
OUT = test_hashes

all: $(OUT)

$(OUT): $(SRC)
	$(NVCC) $(CXXFLAGS) -o $@ $(SRC)

run: $(OUT)
	./$(OUT)

clean:
	rm -f $(OUT)

.PHONY: all run clean
