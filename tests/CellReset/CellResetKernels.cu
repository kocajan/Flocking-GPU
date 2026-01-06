/**
 * \file CellResetKernels.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of CUDA kernels for clearing and rebuilding grid cell ranges. (TESTING)
 *
 * Structure:
 *  - device helper for hash -> (cx,cy,cz) conversion
 *  - kernelResetCells: reset cell metadata and coordinates
 *  - kernelBuildCellRanges: compute [start,end) index ranges per cell
 */

// Minimal structure for testing
struct DeviceGrid {
    int* cellStartBasic; int* cellEndBasic;

    int* cellStartPredator; int* cellEndPredator;

    int* cellStartObstacle; int* cellEndObstacle;

    int* cellX; int* cellY; int* cellZ;

    int numCellsX, numCellsY, numCellsZ;
    int totalCells;
};


__device__ inline void unflattenIndexDevice(int hash, int numCellsX, int numCellsY, int& cX, int& cY, int& cZ) {
    cX = hash % numCellsX;
    int t = hash / numCellsX;
    cY = t % numCellsY;
    cZ = t / numCellsY;
}

__global__ void kernelResetCells(DeviceGrid dGrid) {
    int hash = blockIdx.x * blockDim.x + threadIdx.x;
    if (hash >= dGrid.totalCells) {
        return;
    }

    // Calculate position based on hash
    int cX, cY, cZ;
    unflattenIndexDevice(hash, dGrid.numCellsX, dGrid.numCellsY, cX, cY, cZ);

    // Reset cell ranges
    // - BASIC BOIDS
    dGrid.cellStartBasic[hash] = -1;
    dGrid.cellEndBasic[hash] = -1;

    // - PREDATOR BOIDS
    dGrid.cellStartPredator[hash] = -1;
    dGrid.cellEndPredator[hash] = -1;

    // - OBSTACLE BOIDS
    dGrid.cellStartObstacle[hash] = -1;
    dGrid.cellEndObstacle[hash] = -1;
    
    // Store cell coordinates
    dGrid.cellX[hash] = cX;
    dGrid.cellY[hash] = cY;
    dGrid.cellZ[hash] = cZ;
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
