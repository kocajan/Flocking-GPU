#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaive.hpp"
#include "versions/parallelNaive/ParallelNaiveParameters.hpp"


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


__global__ void simulationStepParallelNaiveKernel(ParallelNaiveParameters::GPUParams params);


void simulationStepParallelNaive(ParallelNaiveParameters& params) {
    // Check for zero boids / zero grid size
    if (params.gpu.dBoids.allBoidCount == 0 || params.cpu.blockSize == 0)
        return;

    // Launch kernel
    int numBlocks = (params.gpu.dBoids.allBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
    simulationStepParallelNaiveKernel<<<numBlocks, params.cpu.blockSize>>>(params.gpu);

    // Check for any CUDA errors
    CHECK_ERROR(cudaPeekAtLastError());
    CHECK_ERROR(cudaDeviceSynchronize());

}
