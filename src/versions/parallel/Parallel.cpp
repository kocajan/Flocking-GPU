#include "versions/parallel/Parallel.hpp"


void simulationStepParallel(ParallelParameters& params) {
    printf("Parallel simulation step\n");
    printf("SimState has %zu boids.\n", params.boids.count);
}
