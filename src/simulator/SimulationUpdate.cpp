#include "simulator/SimulationUpdate.hpp"

// Forward declarations
void regulateBoidPopulation(SimState& simState);
void applyInteraction(SimState& simState, const MouseInteractionEvent& interaction);
void simulationStep(SimState& simState, const SimConfig& simConfig);

void simulationUpdate(SimState& simState, const SimConfig& simConfig, const MouseInteractionEvent& interaction) {
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
