/**
 * \file BoidSorting.cu
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of GPU hash sorting dispatcher for spatial partitioning.
 *
 * Structure:
 *  - power-of-two padding
 *  - padding kernel dispatch
 *  - full bitonic sort sweep (outer k, inner j loops)
 */

#include <limits>
#include <iostream>
#include <cuda_runtime.h>

#include "utils/simStepParallelUtils.cuh"
#include "versions/parallel/kernels/SortKernels.cuh"


void sortBoidsByHash(int boidCount, int* dHash, int* dIndex, int blockSize) {
    if (boidCount <= 1)
        return;

    int N = 1;
    while (N < boidCount) N <<= 1;

    // Launch padding over [0, N) - let the kernel handle out-of-bounds
    if (N > boidCount) {
        int padBlocks = (N + blockSize - 1) / blockSize;

        fillPadding<<<padBlocks, blockSize>>>(dHash, dIndex, boidCount, N, 
                                              std::numeric_limits<int>::max());

        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    int sortBlocks = (N + blockSize - 1) / blockSize;

    for (int outIdx = 2; outIdx <= N; outIdx <<= 1) {
        for (int inIdx = outIdx >> 1; inIdx > 0; inIdx >>= 1) {
            bitonicSortStepKernel<<<sortBlocks, blockSize>>>(
                dHash, dIndex, inIdx, outIdx, N
            );

            CHECK_ERROR(cudaPeekAtLastError());
            CHECK_ERROR(cudaDeviceSynchronize());
        }
    }
}
