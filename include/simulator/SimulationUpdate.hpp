/**
 * \file SimulationUpdate.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief High-level per-frame simulation update routine.
 *
 * Performs one GUI-controlled simulation update step:
 *  - applies mouse / interaction effects
 *  - regulates boid populations toward configured targets
 *  - executes a simulation step when not paused
 *  - advances the simulation tick counter
 */

#pragma once

#include "gui/GUI.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"

/**
 * \brief Perform a full simulation update cycle for one frame.
 *
 * Steps executed in order:
 *  1. Process user interaction and update interaction state
 *  2. Regulate boid counts (may run even when paused)
 *  3. Skip further processing if simulation is paused
 *  4. Execute one simulation step
 *  5. Increment simulation tick counter
 *
 * \param[in,out] simState       Simulation state and boid data.
 * \param[in,out] simConfig      Version-specific configuration.
 * \param[in]     interaction    Mouse interaction event from GUI.
 */
void simulationUpdate(
    SimState& simState,
    Config& simConfig,
    MouseInteractionEvent& interaction
);
