#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#include "versions/parallel/Parallel.hpp"
#include "versions/parallel/ParallelParameters.hpp"
#include "versions/parallel/BoidSorting.hpp"


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


__global__ void kernelComputeHashes(ParallelParameters::GPUParams params, int boidCount, int* dHash, int* dIndex, int* boidIndices);
__global__ void kernelResetCells(ParallelParameters::GPUParams params);
__global__ void kernelBuildCellRanges(int boidCount, int* dHash, int* dCellStart, int* dCellEnd);
__global__ void simulationStepParallelKernel(ParallelParameters::GPUParams params);
__global__ void bitonicSortStepKernel(int* dHash, int* dIndex, int j, int k, int boidCount);


void simulationStepParallel(ParallelParameters& params) {
    // Check for zero boids / zero grid size
    if (params.gpu.dBoids.allBoidCount == 0 || params.cpu.blockSize == 0 || params.gpu.cellSize <= 0.0f || params.gpu.totalCells == 0)
        return;

    // Reset cell ranges
    int numCellBlocks = (params.gpu.totalCells + params.cpu.blockSize - 1) / params.cpu.blockSize;
    kernelResetCells<<<numCellBlocks, params.cpu.blockSize>>>(params.gpu);
    CHECK_ERROR(cudaPeekAtLastError());
    CHECK_ERROR(cudaDeviceSynchronize());

    // Handle basic boids
    int basicBoidCount = static_cast<int>(params.gpu.dBoids.basicBoidCount);
    if (basicBoidCount > 0) {
        // - Determine block size
        int numBasicBoidBlocks = (basicBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
        
        // - Compute hashes
        kernelComputeHashes<<<numBasicBoidBlocks, params.cpu.blockSize>>>(params.gpu, basicBoidCount, 
                                                                          params.gpu.dHashBasic, params.gpu.dIndexBasic, 
                                                                          params.gpu.dBoids.basicBoidIndices);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
        
        // - Sort boids by hash
        sortBoidsByHash(basicBoidCount, params.gpu.dHashBasic, params.gpu.dIndexBasic, params.cpu.blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numBasicBoidBlocks, params.cpu.blockSize>>>(basicBoidCount, 
                                                                            params.gpu.dHashBasic, 
                                                                            params.gpu.dCellStartBasic, 
                                                                            params.gpu.dCellEndBasic);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Handle predator boids
    int predatorBoidCount = static_cast<int>(params.gpu.dBoids.predatorBoidCount);
    if (predatorBoidCount > 0) {
        // - Determine block size
        int numPredatorBoidBlocks = (predatorBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;

        // - Compute hashes
        kernelComputeHashes<<<numPredatorBoidBlocks, params.cpu.blockSize>>>(params.gpu, predatorBoidCount, 
                                                                             params.gpu.dHashPredator, params.gpu.dIndexPredator, 
                                                                             params.gpu.dBoids.predatorBoidIndices);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
        
        // - Sort boids by hash
        sortBoidsByHash(predatorBoidCount, params.gpu.dHashPredator, params.gpu.dIndexPredator, params.cpu.blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numPredatorBoidBlocks, params.cpu.blockSize>>>(predatorBoidCount, 
                                                                               params.gpu.dHashPredator, 
                                                                               params.gpu.dCellStartPredator,
                                                                               params.gpu.dCellEndPredator);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Handle obstacle boids
    int obstacleBoidCount = static_cast<int>(params.gpu.dBoids.obstacleBoidCount);
    if (obstacleBoidCount > 0) {
        // - Determine block size
        int numObstacleBoidBlocks = (obstacleBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
        
        // - Compute hashes
        kernelComputeHashes<<<numObstacleBoidBlocks, params.cpu.blockSize>>>(params.gpu, obstacleBoidCount, 
                                                                             params.gpu.dHashObstacle, params.gpu.dIndexObstacle, 
                                                                             params.gpu.dBoids.obstacleBoidIndices);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
        
        // - Sort boids by hash
        sortBoidsByHash(obstacleBoidCount, params.gpu.dHashObstacle, params.gpu.dIndexObstacle, params.cpu.blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numObstacleBoidBlocks, params.cpu.blockSize>>>(obstacleBoidCount, 
                                                                                params.gpu.dHashObstacle, 
                                                                                params.gpu.dCellStartObstacle,
                                                                                params.gpu.dCellEndObstacle);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Launch the simulation step kernel
    int boidCount = params.gpu.dBoids.allBoidCount;
    if (boidCount > 0) {
        int numBoidBlocks = (boidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
        simulationStepParallelKernel<<<numBoidBlocks, params.cpu.blockSize>>>(params.gpu);
        CHECK_ERROR(cudaPeekAtLastError());   
        CHECK_ERROR(cudaDeviceSynchronize());
    }
}
