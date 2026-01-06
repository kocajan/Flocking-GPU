/**
 * \file SimulationStep.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Dispatch entry point for running a single simulation step.
 *
 * Selects and executes the active simulation implementation based on
 * the `version` parameter stored in the simulation state.
 */

#pragma once

#include "core/SimState.hpp"
#include "config/Config.hpp"

/**
 * \brief Execute one simulation step using the selected implementation.
 *
 * The implementation is chosen dynamically according to:
 *   simState.version.string()
 *
 * Supported versions include (but are not limited to):
 *  - sequentialNaive
 *  - sequential
 *  - parallelNaive
 *  - parallel
 *
 * Each implementation defines its own parameter wrapper and update routine.
 *
 * \param[in,out] simState Simulation state and population data.
 * \param[in,out] simConfig Version-dependent configuration values.
 */
void simulationStep(SimState& simState, Config& simConfig);
