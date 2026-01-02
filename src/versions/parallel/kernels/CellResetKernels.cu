#include "versions/parallel/ParallelParameters.hpp"


__device__ inline void unflattenIndexDevice(int hash, int numCellsX, int numCellsY, int& cX, int& cY, int& cZ) {
    cX = hash % numCellsX;
    int t = hash / numCellsX;
    cY = t % numCellsY;
    cZ = t / numCellsY;
}

__global__ void kernelResetCells(ParallelParameters::GPUParams params) {
    int hash = blockIdx.x * blockDim.x + threadIdx.x;
    if (hash >= params.totalCells) {
        return;
    }

    // Calculate position based on hash
    int cX, cY, cZ;
    unflattenIndexDevice(hash, params.numCellsX, params.numCellsY, cX, cY, cZ);

    if (params.is2D) {
        cZ = 0;
    }

    params.dCellStartBasic[hash] = -1;
    params.dCellEndBasic[hash] = -1;
    params.dCellStartPredator[hash] = -1;
    params.dCellEndPredator[hash] = -1;
    params.dCellStartObstacle[hash] = -1;
    params.dCellEndObstacle[hash] = -1;
    
    params.dCellX[hash] = cX;
    params.dCellY[hash] = cY;
    params.dCellZ[hash] = cZ;
}

__global__ void kernelBuildCellRanges(int boidCount, int* dHash, int* dCellStart, int* dCellEnd) {
    // Compute global boid index
    int currentBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;

    if (currentBoidIdx >= boidCount) {
        return;
    }

    // Get current boid's cell hash
    int hash = dHash[currentBoidIdx];

    // Handle start of cell
    if (currentBoidIdx == 0 || hash != dHash[currentBoidIdx - 1]) {
        dCellStart[hash] = currentBoidIdx;
    }

    // Handle end of cell
    if (currentBoidIdx == boidCount - 1 || hash != dHash[currentBoidIdx + 1]) {
        dCellEnd[hash] = currentBoidIdx + 1; // end is exclusive
    }
}
