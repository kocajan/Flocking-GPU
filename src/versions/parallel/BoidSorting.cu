#include <limits>
#include <iostream>
#include <cuda_runtime.h>


namespace {
    #define CHECK_ERROR( error ) ( HandleError( error, __FILE__, __LINE__ ) )

    static void HandleError(cudaError_t error, const char* file, int line) { 
        if (error != cudaSuccess) { 
            std::cout << cudaGetErrorString(error) << " in " << file << " at line " << line << std::endl; 
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

    // pad tail
    if (N > boidCount) {
        int padBlocks = (N - boidCount + blockSize - 1) / blockSize;

        fillPadding<<<padBlocks, blockSize>>>(dHash, dIndex, boidCount, N, std::numeric_limits<int>::max());

        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    int sortBlocks = (N + blockSize - 1) / blockSize;

    for (int k = 2; k <= N; k <<= 1) {
        for (int j = k >> 1; j > 0; j >>= 1) {
            bitonicSortStepKernel<<<sortBlocks, blockSize>>>(
                dHash, dIndex, j, k, boidCount
            );
            CHECK_ERROR(cudaPeekAtLastError());
            CHECK_ERROR(cudaDeviceSynchronize());
        }
    }
}
