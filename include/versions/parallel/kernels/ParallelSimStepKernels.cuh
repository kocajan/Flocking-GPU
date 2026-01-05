/**
 * \file ParallelSimStepKernels.cuh
 * \author Jan Koča
 * \date 01-05-2026
 * \brief CUDA kernels for the optimized parallel simulation step.
 *
 * Kernels operate on:
 *  - spatially-sorted boid data
 *  - neighborhood cell ranges in the spatial grid
 *
 * Responsibilities:
 *  - resolve flocking behavior inside local neighborhoods
 *  - apply world + interaction + physics rules
 *  - store updated accelerations
 */

#include "versions/parallel/ParallelParameters.cuh"

/**
 * \brief Parallel simulation step for basic boids.
 *
 * Each thread:
 *  - processes one basic boid
 *  - queries neighbors through spatial grid lookup
 *  - applies flocking and steering forces
 *  - resolves collisions and physics
 *
 * \param[in,out] params GPU simulation parameter block.
 */
__global__ void simulationStepParallelBasicBoidsKernel(
    ParallelParameters::GPUParams params
);

/**
 * \brief Parallel simulation step for predator boids.
 *
 * Each thread:
 *  - processes one predator boid
 *  - performs neighborhood queries via spatial grid
 *  - executes chasing / stamina / rest logic
 *  - resolves motion and collisions
 *
 * \param[in,out] params GPU simulation parameter block.
 */
__global__ void simulationStepParallelPredatorBoidsKernel(
    ParallelParameters::GPUParams params
);
