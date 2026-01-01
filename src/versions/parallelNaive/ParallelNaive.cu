#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaive.hpp"
#include "versions/parallelNaive/ParallelNaiveParameters.hpp"


__global__ void simulationStepParallelNaiveKernel(ParallelNaiveParameters::GPUParams params);


void simulationStepParallelNaive(ParallelNaiveParameters& params) {
    // Check for zero boids / zero grid size
    if (params.gpu.boidCount == 0 || params.cpu.gridSize == 0)
        return;

    // Launch kernel
    simulationStepParallelNaiveKernel<<<params.cpu.gridSize, params.cpu.blockSize>>>(params.gpu);

}
