NVCC ?= nvcc
CXXFLAGS := -std=c++17

SRC = main.cu BoidSorting.cu SortKernels.cu
OUT = test_sort

all: $(OUT)

$(OUT): $(SRC)
	$(NVCC) $(CXXFLAGS) -o $@ $(SRC)

run: $(OUT)
	./$(OUT)

clean:
	rm -f $(OUT)

.PHONY: all run clean
