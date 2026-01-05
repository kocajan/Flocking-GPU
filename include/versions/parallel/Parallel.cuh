/**
 * \file Parallel.cuh
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Interface for the optimized parallel simulation step.
 *
 * Runs one simulation iteration on the GPU using:
 *  - spatial hashing and sorting
 *  - neighborhood grid lookup
 *  - specialized kernels for basic and predator boids
 */

#pragma once

#include "versions/parallel/ParallelParameters.cuh"

/**
 * \brief Execute one parallel simulation step.
 *
 * High-level pipeline:
 *  1) reset grid cell ranges
 *  2) hash boids into spatial cells
 *  3) sort boids by hash
 *  4) build per-cell ranges
 *  5) run simulation kernels
 *
 * \param[in,out] params Simulation state and GPU resources.
 */
void simulationStepParallel(ParallelParameters& params);
