/**
 * \file SortKernels.cu
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of CUDA padding and bitonic sort kernels for hash/index buffers.
 *
 * Structure:
 *  - fillPadding: marks padded region beyond boid count with sentinel values
 *  - bitonicSortStepKernel: one compare-swap step of bitonic network
 */

#include <cuda_runtime.h>


__global__ void fillPadding(int* dHash, int* dIndex, int boidCount, int N, int sentinel) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    // Only write padding in [boidCount, N)
    if (i < boidCount || i >= N) return;

    dHash[i]  = sentinel;
    dIndex[i] = -1; 
}

// One step of the bitonic sort network over N elements.
__global__ void bitonicSortStepKernel(int* dHash, int* dIndex, int j, int k, int N) {
    unsigned int i   = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (unsigned int)N) return;

    unsigned int ixj = i ^ j;
    if (ixj >= (unsigned int)N) return;

    if (ixj > i) {
        bool ascending = ((i & k) == 0);

        int key_i   = dHash[i];
        int key_ixj = dHash[ixj];

        bool doSwap =
            (ascending && key_i > key_ixj) ||
            (!ascending && key_i < key_ixj);

        if (doSwap) {
            // swap keys
            dHash[i]  = key_ixj;
            dHash[ixj] = key_i;

            // swap associated indices
            int val_i   = dIndex[i];
            int val_ixj = dIndex[ixj];
            dIndex[i]   = val_ixj;
            dIndex[ixj] = val_i;
        }
    }
}
