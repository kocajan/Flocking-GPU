#include "versions/parallelNaive/ParallelNaive.hpp"


void parallelNaiveSimulationStep(ParallelNaiveParameters& params) {
    printf("Parallel Naive simulation step\n");
    printf("SimState tick: %lu\n", params.boids.size());
}
