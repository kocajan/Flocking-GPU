/**
 * \file SimulationUpdate.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of the per-frame simulation update routine.
 */

#include "simulator/SimulationUpdate.hpp"

#include "simulator/SimulationStep.hpp"
#include "simulator/InteractionSystem.hpp"
#include "simulator/BoidPopulationRegulation.hpp"


void simulationUpdate(SimState& simState, Config& simConfig, 
                      MouseInteractionEvent& interaction) {
    // Apply GUI and input interactions
    applyInteraction(simState, interaction);

    // Regulate population (allowed while paused)
    regulateBoidPopulation(simState);

    // Pause gate
    if (simState.paused.binary())
        return;

    // Run simulation
    simulationStep(simState, simConfig);

    // Advance simulation time
    simState.tick++;
}
