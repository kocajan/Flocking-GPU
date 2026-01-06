/**
 * \file Parallel.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of the optimized parallel simulation step.
 *
 * Orchestration code coordinating:
 *  - grid reset and hashing
 *  - bitonic sorting by cell hash
 *  - cell range construction
 *  - execution of simulation kernels
 */

#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#include "core/DeviceStructures.hpp"
#include "versions/parallel/Parallel.cuh"
#include "utils/simStepParallelUtils.cuh"
#include "versions/parallel/BoidSorting.cuh"
#include "versions/parallel/ParallelParameters.cuh"
#include "versions/parallel/kernels/HashesKernels.cuh"
#include "versions/parallel/kernels/CellResetKernels.cuh"
#include "versions/parallel/kernels/ParallelSimStepKernels.cuh"


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

    // Handle basic boids
    int basicBoidCount = dBoids.basicBoidCount;
    if (basicBoidCount > 0) {
        // - Determine number of blocks
        int numBasicBoidBlocks = (basicBoidCount + blockSize - 1) / blockSize;
        
        // - Compute hashes
        kernelComputeHashes<<<numBasicBoidBlocks, blockSize>>>(params.gpu, 
                                                               dBoids.posBasic, basicBoidCount, 
                                                               dGrid.hashBasic, dGrid.indexBasic);
        CHECK_ERROR(cudaPeekAtLastError());
        
        // - Sort boids by hash
        sortBoidsByHash(basicBoidCount, dGrid.hashBasic, dGrid.indexBasic, blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numBasicBoidBlocks, blockSize>>>(basicBoidCount, dGrid.hashBasic, 
                                                                 dGrid.cellStartBasic, dGrid.cellEndBasic);
        CHECK_ERROR(cudaPeekAtLastError());
    }

    // Handle predator boids
    int predatorBoidCount = dBoids.predatorBoidCount;
    if (predatorBoidCount > 0) {
        // - Determine number of blocks
        int numPredatorBoidBlocks = (predatorBoidCount + blockSize - 1) / blockSize;

        // - Compute hashes
        kernelComputeHashes<<<numPredatorBoidBlocks, blockSize>>>(params.gpu, 
                                                                 dBoids.posPredator, predatorBoidCount, 
                                                                 dGrid.hashPredator, dGrid.indexPredator);
        CHECK_ERROR(cudaPeekAtLastError());
        
        // - Sort boids by hash
        sortBoidsByHash(predatorBoidCount, dGrid.hashPredator, dGrid.indexPredator, blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numPredatorBoidBlocks, blockSize>>>(predatorBoidCount, 
                                                                     dGrid.hashPredator, 
                                                                     dGrid.cellStartPredator,
                                                                     dGrid.cellEndPredator);
        CHECK_ERROR(cudaPeekAtLastError());
    }

    // Handle obstacle boids
    int obstacleBoidCount = dBoids.obstacleBoidCount;
    if (obstacleBoidCount > 0) {
        // - Determine number of blocks
        int numObstacleBoidBlocks = (obstacleBoidCount + blockSize - 1) / blockSize;
        
        // - Compute hashes
        kernelComputeHashes<<<numObstacleBoidBlocks, blockSize>>>(params.gpu, 
                                                                  dBoids.posObstacle, obstacleBoidCount, 
                                                                  dGrid.hashObstacle, dGrid.indexObstacle);
        CHECK_ERROR(cudaPeekAtLastError());
        
        // - Sort boids by hash
        sortBoidsByHash(obstacleBoidCount, dGrid.hashObstacle, dGrid.indexObstacle, blockSize);

        // - Build cell ranges
        kernelBuildCellRanges<<<numObstacleBoidBlocks, blockSize>>>(obstacleBoidCount, 
                                                                    dGrid.hashObstacle, 
                                                                    dGrid.cellStartObstacle,
                                                                    dGrid.cellEndObstacle);
        CHECK_ERROR(cudaPeekAtLastError());
    }

    // Launch the simulation step kernel for basic and predator boids
    if (basicBoidCount > 0) {
        int numBasicBoidBlocks = (basicBoidCount + blockSize - 1) / blockSize;
        simulationStepParallelBasicBoidsKernel<<<numBasicBoidBlocks, blockSize>>>(params.gpu);
        CHECK_ERROR(cudaPeekAtLastError());   
    }

    if (predatorBoidCount > 0) {
        int numPredatorBoidBlocks = (predatorBoidCount + blockSize - 1) / blockSize;
        simulationStepParallelPredatorBoidsKernel<<<numPredatorBoidBlocks, blockSize>>>(params.gpu);
        CHECK_ERROR(cudaPeekAtLastError());   
    }
}
