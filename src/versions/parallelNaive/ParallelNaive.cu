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


__global__ void simulationStepParallelNaiveBasicBoidsKernel(ParallelNaiveParameters::GPUParams params);
__global__ void simulationStepParallelNaivePredatorBoidsKernel(ParallelNaiveParameters::GPUParams params);


void simulationStepParallelNaive(ParallelNaiveParameters& params) {
    // Check valid block size
    if (params.cpu.blockSize <= 0) {
        std::cout << "Warning: Invalid block size for Parallel Naive simulation step: " << params.cpu.blockSize << std::endl;
        return;
    }

    // Process basic boids
    if (params.gpu.dBoids.basicBoidCount > 0) {
        int numBlocksBasic = (params.gpu.dBoids.basicBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
        simulationStepParallelNaiveBasicBoidsKernel<<<numBlocksBasic, params.cpu.blockSize>>>(params.gpu);

        // Check for any CUDA errors
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Process predator boids
    if (params.gpu.dBoids.predatorBoidCount > 0) {
        int numBlocksPredator = (params.gpu.dBoids.predatorBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
        simulationStepParallelNaivePredatorBoidsKernel<<<numBlocksPredator, params.cpu.blockSize>>>(params.gpu);

        // Check for any CUDA errors
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

}
