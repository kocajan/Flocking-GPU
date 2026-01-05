/**
 * \file BoidPopulationRegulation.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief High-level population regulation entry point.
 *
 * Applies population regulation for each boid category using target values
 * from the simulation state. Obstacle boids may be cleared fully when the
 * `deleteObstacles` flag is active in SimState.
 */

#pragma once

#include "core/SimState.hpp"

/**
 * \brief Regulate boid populations according to configured targets.
 *
 * Performs:
 *  - gradual regulation of Basic boids toward basicBoidCountTarget
 *  - gradual regulation of Predator boids toward predatorBoidCountTarget
 *  - full removal of obstacles when deleteObstacles is enabled in SimState
 *
 * Internally delegates spawning / deletion logic to
 * `BoidPopulationRegulationUtils`.
 *
 * \param[in,out] simulationState Simulation state and boid storage.
 */
void regulateBoidPopulation(SimState& simulationState);
