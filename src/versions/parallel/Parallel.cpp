#include "versions/parallel/Parallel.hpp"


void simulationStepParallel(ParallelParameters& params) {
    printf("Parallel simulation step\n");
    printf("SimState tick: %lu\n", params.boids.size());
}
