/**
 * \file ParallelNaiveSimStepKernels.cuh
 * \author Jan Koča
 * \date 05-01-2026
 * \brief CUDA kernels for the parallel-naive flocking simulation step.
 *
 * This header declares:
 *  - basic-boid update kernel
 *  - predator-boid update kernel
 *
 * Each kernel:
 *  - processes one boid per thread
 *  - computes behavior and interactions
 *  - applies dynamics and collisions
 */

#pragma once

#include "versions/parallelNaive/ParallelNaiveParameters.cuh"

/**
 * \brief Update basic boids in the parallel-naive GPU version.
 *
 * One thread corresponds to one basic boid. The kernel:
 *  - resolves flocking behavior
 *  - applies interactions and dynamics
 *  - runs collision handling
 *
 * \param[in,out] params GPU runtime parameter block.
 */
__global__ void simulationStepParallelNaiveBasicBoidsKernel(
    ParallelNaiveParameters::GPUParams params
);

/**
 * \brief Update predator boids in the parallel-naive GPU version.
 *
 * One thread corresponds to one predator boid. The kernel:
 *  - resolves chasing behavior and stamina logic
 *  - applies interactions and dynamics
 *  - runs collision handling
 *
 * \param[in,out] params GPU runtime parameter block.
 */
__global__ void simulationStepParallelNaivePredatorBoidsKernel(
    ParallelNaiveParameters::GPUParams params
);
