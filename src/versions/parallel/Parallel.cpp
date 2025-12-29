#include "versions/parallel/Parallel.hpp"


void parallelSimulationStep(ParallelParameters& params) {
    printf("Parallel simulation step\n");
    printf("SimState tick: %lu\n", params.boids.size());
}
