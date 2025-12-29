#include "versions/parallelNaive/ParallelNaive.hpp"


void parallelNaiveSimulationStep(SimState& simState, const Config& simConfig) {
    printf("Parallel Naive simulation step\n");
    printf("SimState tick: %lu\n", simState.tick);
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
