#include "versions/sequential/Sequential.hpp"


void sequentialSimulationStep(SequentialParameters& params) {
    printf("Sequential simulation step\n");
    printf("SimState tick: %lu\n", params.boids.size());
}
