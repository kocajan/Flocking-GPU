/**
 * \file BoidSorting.cu
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of GPU hash sorting dispatcher for spatial partitioning (TESTING).
 *
 * Structure:
 *  - power-of-two padding
 *  - padding kernel dispatch
 *  - full bitonic sort sweep (outer k, inner j loops)
 */

#include <limits>
#include <iostream>
#include <cuda_runtime.h>


namespace {
    #define CHECK_ERROR( error ) ( HandleError( error, __FILE__, __LINE__ ) )

    static void HandleError(cudaError_t error, const char* file, int line) { 
        if (error != cudaSuccess) { 
            std::cout << cudaGetErrorString(error) << " in " << file
                      << " at line " << line << std::endl; 
            int w = scanf(" "); 
            exit(EXIT_FAILURE); 
        } 
    }
} // anonymous namespace


__global__ void fillPadding(int* dHash, int* dIndex, int boidCount, int N, int sentinel);
__global__ void bitonicSortStepKernel(int* dHash, int* dIndex, int j, int k, int N);


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
