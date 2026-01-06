/**
 * \file InteractionSystem.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Mouse-driven runtime interaction handling for simulation state.
 *
 * Maps GUI mouse events to world-space actions such as:
 * - spawning / deleting boids
 * - drawing / erasing obstacles
 * - predator and obstacle population changes
 * - attract / repel interaction effects
 */

#pragma once

#include "gui/GUI.hpp"
#include "core/SimState.hpp"

/**
 * \brief Apply a mouse interaction event to the simulation state.
 *
 * Interprets effect strings stored in SimState interaction parameters and
 * performs the associated action (e.g., spawn, delete) or sets up interaction
 * effects for the simulation step. (e.g., attract / repel).
 *
 * \param[in,out] simState Simulation state to modify.
 * \param[in] interaction GUI-reported mouse interaction event.
 */
void applyInteraction(SimState& simState, MouseInteractionEvent& interaction);
