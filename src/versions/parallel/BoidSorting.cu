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

    // pad tail: launch over FULL [0, N) and let kernel decide which to write
    if (N > boidCount) {
        int padBlocks = (N + blockSize - 1) / blockSize;

        fillPadding<<<padBlocks, blockSize>>>(
            dHash, dIndex, boidCount, N, std::numeric_limits<int>::max()
        );

        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    int sortBlocks = (N + blockSize - 1) / blockSize;

    for (int k = 2; k <= N; k <<= 1) {
        for (int j = k >> 1; j > 0; j >>= 1) {
            bitonicSortStepKernel<<<sortBlocks, blockSize>>>(
                dHash, dIndex, j, k, N
            );

            CHECK_ERROR(cudaPeekAtLastError());
            CHECK_ERROR(cudaDeviceSynchronize());
        }
    }
}