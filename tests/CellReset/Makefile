NVCC ?= nvcc
CXXFLAGS := -std=c++17

SRC = main.cu CellResetKernels.cu
OUT = test_kernels

all: $(OUT)

$(OUT): $(SRC)
	$(NVCC) $(CXXFLAGS) -o $@ $(SRC)

run: $(OUT)
	./$(OUT)

clean:
	rm -f $(OUT)

.PHONY: all run clean
