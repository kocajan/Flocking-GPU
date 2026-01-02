#include <cuda_runtime.h>
#include <iostream>


// One step of the bitonic sort network.
__global__ void bitonicSortStepKernel(int* dHash, int* dIndex, int  j, int  k, int  count) {
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

    // We conceptually sort up to N = next power of two >= count,
    // but only indices < count are real. Others are treated as +∞
    // by simply skipping any operation touching them.
    unsigned int ixj = i ^ j;  // partner index in this step

    if (i >= (unsigned int)count || ixj >= (unsigned int)count)
        return;

    if (ixj > i) {
        bool ascending = ((i & k) == 0);

        int key_i   = dHash[i];
        int key_ixj = dHash[ixj];

        bool doSwap =
            (ascending && key_i > key_ixj) ||
            (!ascending && key_i < key_ixj);

        if (doSwap) {
            // swap keys
            dHash[i] = key_ixj;
            dHash[ixj] = key_i;

            // swap associated indices
            int val_i = dIndex[i];
            int val_ixj = dIndex[ixj];
            dIndex[i] = val_ixj;
            dIndex[ixj] = val_i;
        }
    }
}

__global__ void fillPadding(int* dHash, int* dIndex, int boidCount, int N, int sentinel) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < boidCount || i >= N) return;  // only touch [boidCount, N)

    dHash[i]  = sentinel;
    dIndex[i] = -1;
}

