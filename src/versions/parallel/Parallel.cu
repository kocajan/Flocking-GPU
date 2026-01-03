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

__global__ void simulationStepParallelBasicBoidsKernel(ParallelParameters::GPUParams params);
__global__ void simulationStepParallelPredatorBoidsKernel(ParallelParameters::GPUParams params);

__global__ void kernelComputeHashes(ParallelParameters::GPUParams params, Vec3* dPos, int boidCount, int* dHash, int* dIndex);
__global__ void kernelResetCells(DeviceGrid dGrid);
__global__ void kernelBuildCellRanges(int boidCount, int* dHash, int* dCellStart, int* dCellEnd);
__global__ void bitonicSortStepKernel(int* dHash, int* dIndex, int j, int k, int boidCount);


void simulationStepParallel(ParallelParameters& params) {
    // Check for zero boids / zero grid size
    if (params.cpu.blockSize == 0 || params.gpu.dGrid.cellSize <= 0.0f || params.gpu.dGrid.totalCells == 0)
        return;

    // Unpack some parameters for easier access
    DeviceBoids& dBoids = params.gpu.dBoids;
    DeviceGrid& dGrid = params.gpu.dGrid;
    int blockSize = params.cpu.blockSize;

    // Reset cell ranges
    int numCellBlocks = (dGrid.totalCells + params.cpu.blockSize - 1) / params.cpu.blockSize;
    kernelResetCells<<<numCellBlocks, params.cpu.blockSize>>>(dGrid);
    CHECK_ERROR(cudaPeekAtLastError());
    CHECK_ERROR(cudaDeviceSynchronize());

    // Handle basic boids
    int basicBoidCount = dBoids.basicBoidCount;
    if (basicBoidCount > 0) {
        // - Determine block size
        int numBasicBoidBlocks = (basicBoidCount + blockSize - 1) / blockSize;
        
        // - Compute hashes
        kernelComputeHashes<<<numBasicBoidBlocks, blockSize>>>(params.gpu, 
                                                               dBoids.posBasic, basicBoidCount, 
                                                               dGrid.hashBasic, dGrid.indexBasic);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
        
        // - Sort boids by hash
        sortBoidsByHash(basicBoidCount, dGrid.hashBasic, dGrid.indexBasic, blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numBasicBoidBlocks, blockSize>>>(basicBoidCount, dGrid.hashBasic, 
                                                                 dGrid.cellStartBasic, dGrid.cellEndBasic);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Handle predator boids
    int predatorBoidCount = dBoids.predatorBoidCount;
    if (predatorBoidCount > 0) {
        // - Determine block size
        int numPredatorBoidBlocks = (predatorBoidCount + blockSize - 1) / blockSize;

        // - Compute hashes
        kernelComputeHashes<<<numPredatorBoidBlocks, blockSize>>>(params.gpu, 
                                                                 dBoids.posPredator, predatorBoidCount, 
                                                                 dGrid.hashPredator, dGrid.indexPredator);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
        
        // - Sort boids by hash
        sortBoidsByHash(predatorBoidCount, dGrid.hashPredator, dGrid.indexPredator, blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numPredatorBoidBlocks, blockSize>>>(predatorBoidCount, 
                                                                     dGrid.hashPredator, 
                                                                     dGrid.cellStartPredator,
                                                                     dGrid.cellEndPredator);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Handle obstacle boids
    int obstacleBoidCount = dBoids.obstacleBoidCount;
    if (obstacleBoidCount > 0) {
        // - Determine block size
        int numObstacleBoidBlocks = (obstacleBoidCount + blockSize - 1) / blockSize;
        
        // - Compute hashes
        kernelComputeHashes<<<numObstacleBoidBlocks, blockSize>>>(params.gpu, 
                                                                  dBoids.posObstacle, obstacleBoidCount, 
                                                                  dGrid.hashObstacle, dGrid.indexObstacle);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
        
        // - Sort boids by hash
        sortBoidsByHash(obstacleBoidCount, dGrid.hashObstacle, dGrid.indexObstacle, blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numObstacleBoidBlocks, blockSize>>>(obstacleBoidCount, 
                                                                    dGrid.hashObstacle, 
                                                                    dGrid.cellStartObstacle,
                                                                    dGrid.cellEndObstacle);
        CHECK_ERROR(cudaPeekAtLastError());
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    // Launch the simulation step kernel for basic and predator boids
    if (basicBoidCount > 0) {
        int numBasicBoidBlocks = (basicBoidCount + blockSize - 1) / blockSize;
        simulationStepParallelBasicBoidsKernel<<<numBasicBoidBlocks, blockSize>>>(params.gpu);
        CHECK_ERROR(cudaPeekAtLastError());   
        CHECK_ERROR(cudaDeviceSynchronize());
    }

    if (predatorBoidCount > 0) {
        int numPredatorBoidBlocks = (predatorBoidCount + blockSize - 1) / blockSize;
        simulationStepParallelPredatorBoidsKernel<<<numPredatorBoidBlocks, blockSize>>>(params.gpu);
        CHECK_ERROR(cudaPeekAtLastError());   
        CHECK_ERROR(cudaDeviceSynchronize());
    }
}
