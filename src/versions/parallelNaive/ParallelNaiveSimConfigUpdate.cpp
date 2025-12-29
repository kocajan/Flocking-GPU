#include "versions/parallelNaive/ParallelNaiveSimConfigUpdate.hpp"


void parallelNaiveSimConfigUpdate(Config& simConfig) {
    // Currently no specific updates needed for parallelNaive version
    printf("Parallel Naive simulation config update\n");
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
