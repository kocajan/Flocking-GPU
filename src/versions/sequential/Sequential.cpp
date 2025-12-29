#include "versions/sequential/Sequential.hpp"


void sequentialSimulationStep(SimState& simState, const Config& simConfig) {
    printf("Sequential simulation step\n");
    printf("SimState tick: %lu\n", simState.tick);
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
