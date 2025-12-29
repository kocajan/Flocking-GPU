#include "versions/sequential/SequentialSimConfigUpdate.hpp"


void sequentialSimConfigUpdate(Config& simConfig) {
    // Currently no specific updates needed for sequential version
    printf("Sequential simulation config update\n");
    printf("Bounce: %d\n", simConfig.binary("bounce"));
}
