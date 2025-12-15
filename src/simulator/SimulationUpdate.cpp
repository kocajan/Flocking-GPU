#include "simulator/SimulationUpdate.hpp"

// Forward declarations
void regulateBoidPopulation(SimState& simState);
void applyInteractions(SimState& simState, const std::vector<InteractionEvent>& interactions);
void simulationStep(SimState& simState, const SimConfig& simConfig);

void simulationUpdate(SimState& simState, const SimConfig& simConfig, const std::vector<InteractionEvent>& interactions) {
    // Apply GUI and input interactions
    applyInteractions(simState, interactions);

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
