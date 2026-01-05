/**
 * \file ParallelNaive.cuh
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Parallel-naive GPU implementation of the flocking simulation.
 *
 * This version:
 *  - runs boid updates on the GPU
 *  - launches one CUDA kernel per boid type
 *  - performs neighborhood interaction in O(N^2) - in parallel
 */

#pragma once

#include "versions/parallelNaive/ParallelNaiveParameters.cuh"

/**
 * \brief Execute one full simulation step of the parallel-naive GPU version.
 *
 * The function:
 *  - validates scheduling parameters
 *  - launches basic-boid kernel
 *  - launches predator-boid kernel
 *  - synchronizes after each launch
 *
 * \param[in,out] params CPU–GPU runtime parameter block.
 */
void simulationStepParallelNaive(ParallelNaiveParameters& params);
