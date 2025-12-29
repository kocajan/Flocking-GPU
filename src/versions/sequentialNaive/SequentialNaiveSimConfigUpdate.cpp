#include "versions/sequentialNaive/SequentialNaiveSimConfigUpdate.hpp"

void sequentialNaiveSimConfigUpdate(Config& simConfig) {
    // Currently no specific updates needed for sequentialNaive version
    printf("Sequential Naive simulation config update\n");
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
