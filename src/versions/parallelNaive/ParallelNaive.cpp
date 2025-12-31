#include "versions/parallelNaive/ParallelNaive.hpp"


void simulationStepParallelNaive(ParallelNaiveParameters& params) {
    printf("Parallel Naive simulation step\n");
    printf("SimState has %zu boids.\n", params.boids.count);
}
