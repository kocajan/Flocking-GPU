#include "simulator/SimulationUpdate.hpp"

#include "simulator/SimulationStep.hpp"
#include "simulator/InteractionSystem.hpp"
#include "simulator/BoidPopulationRegulation.hpp"


void simulationUpdate(SimState& simState, Config& simConfig, MouseInteractionEvent& interaction) {
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
