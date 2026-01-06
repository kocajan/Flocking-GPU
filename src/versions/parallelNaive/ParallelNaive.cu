/**
 * \file ParallelNaive.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of the parallel-naive GPU flocking simulation step.
 *
 * Structure:
 *  - kernel launch scheduling
 *  - per-type kernel execution
 *  - CUDA error checking and synchronization
 */

#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#include "utils/simStepParallelUtils.cuh"
#include "versions/parallelNaive/ParallelNaive.cuh"
#include "versions/parallelNaive/ParallelNaiveParameters.cuh"
#include "versions/parallelNaive/kernels/ParallelNaiveSimStepKernels.cuh"


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
    }

    // Process predator boids
    if (params.gpu.dBoids.predatorBoidCount > 0) {
        int numBlocksPredator = (params.gpu.dBoids.predatorBoidCount + params.cpu.blockSize - 1) / params.cpu.blockSize;
        simulationStepParallelNaivePredatorBoidsKernel<<<numBlocksPredator, params.cpu.blockSize>>>(params.gpu);

        // Check for any CUDA errors
        CHECK_ERROR(cudaPeekAtLastError());
    }
}
