#include "versions/parallel/Parallel.hpp"


void parallelSimulationStep(SimState& simState, const Config& simConfig) {
    printf("Parallel simulation step\n");
    printf("SimState tick: %lu\n", simState.tick);
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
