#include "versions/parallel/ParallelSimConfigUpdate.hpp"

void parallelSimConfigUpdate(Config& simConfig) {
    // Currently no specific updates needed for parallel version
    printf("Parallel simulation config update\n");
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
