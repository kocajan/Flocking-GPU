/**
 * \file SequentialNaive.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Sequential CPU baseline implementation of the flocking simulation.
 *
 * This version:
 *  - iterates boids in a single thread
 *  - computes neighborhood interactions in O(N^2)
 */

#pragma once

#include "core/SimState.hpp"
#include "config/Config.hpp"
#include "versions/sequentialNaive/SequentialNaiveParameters.hpp"

/**
 * \brief Execute one full simulation step of the sequential-naive version.
 *
 * The function:
 *  - updates basic boids
 *  - updates predator boids
 *  - applies dynamics, collisions, and interactions
 *
 * \param[in,out] params Runtime simulation parameters and data buffers.
 */
void simulationStepSequentialNaive(SequentialNaiveParameters& params);
