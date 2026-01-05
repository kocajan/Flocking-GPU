/**
 * \file CellResetKernels.cu
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of CUDA kernels for clearing and rebuilding grid cell ranges.
 *
 * Structure:
 *  - device helper for hash → (cx,cy,cz) conversion
 *  - kernelResetCells: reset cell metadata and coordinates
 *  - kernelBuildCellRanges: compute [start,end) index ranges per cell
 */

#include "core/DeviceStructures.hpp"
#include "versions/parallel/ParallelParameters.cuh"
#include "versions/parallel/kernels/CellResetKernels.cuh"


__device__ inline void unflattenIndexDevice(int hash, int numCellsX, int numCellsY, int& cX, int& cY, int& cZ);


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

/**
 * \brief Convert flattened cell hash index into 3D cell coordinates.
 *
 * \param[in] hash Linear cell index.
 * \param[in] numCellsX Cell count along X.
 * \param[in] numCellsY Cell count along Y.
 * \param[in, out] cX Output cell index X.
 * \param[in, out] cY Output cell index Y.
 * \param[in, out] cZ Output cell index Z.
 */
__device__ inline void unflattenIndexDevice(int hash, int numCellsX, int numCellsY, int& cX, int& cY, int& cZ) {
    cX = hash % numCellsX;
    int t = hash / numCellsX;
    cY = t % numCellsY;
    cZ = t / numCellsY;
}
