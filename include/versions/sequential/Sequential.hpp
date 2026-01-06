/**
 * \file Sequential.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Optimized sequential CPU implementation of the flocking simulation.
 *
 * This version:
 *  - uses a spatial hashing grid for neighborhood queries
 *  - reduces interaction cost compared to the naive O(N^2) variant
 *  - iterates boids in a single thread
 */

#pragma once

#include "versions/sequential/SequentialParameters.hpp"

/**
 * \brief Execute one full simulation step of the optimized sequential version.
 *
 * The function:
 *  - rebuilds the spatial grid
 *  - updates basic boids
 *  - updates predator boids
 *  - applies interactions, dynamics, and collisions
 *
 * \param[in,out] params Runtime simulation parameters and data buffers.
 */
void simulationStepSequential(SequentialParameters& params);
