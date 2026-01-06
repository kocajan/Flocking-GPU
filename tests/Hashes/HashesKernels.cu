/**
 * \file HashesKernels.cu
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of CUDA kernels for computing spatial hashing of boids. (TESTING)
 *
 * Structure:
 *  - device helpers mirroring SpatialGrid CPU logic
 *  - kernelComputeHashes: world position -> cell index -> hash
 */

__device__ inline int flattenIndexDevice(int cX, int cY, int cZ, int numCellsX, int numCellsY);
__device__ inline int worldToCellIndexDevice(float pos, float worldSize, int cellCount, float cellSize, bool bounce);


// Minimal structures for the tests
struct Vec3 { float x,y,z; };

struct DeviceGrid {
    float cellSize;
    int numCellsX;
    int numCellsY;
    int numCellsZ;
};

struct GPUParams {
    DeviceGrid dGrid;
    bool is2D;
    bool bounce;
    float worldX;
    float worldY;
    float worldZ;
};


__global__ void kernelComputeHashes(GPUParams params, Vec3* dPos, int boidCount, int* dHash, int* dIndex) {
    // Compute global boid index
    int currentBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (currentBoidIdx >= boidCount) {
        return;
    }

    // Get current boids position
    const Vec3 pos = dPos[currentBoidIdx];

    // Compute cell indices
    int cX = worldToCellIndexDevice(pos.x, params.worldX, params.dGrid.numCellsX, params.dGrid.cellSize, params.bounce);
    int cY = worldToCellIndexDevice(pos.y, params.worldY, params.dGrid.numCellsY, params.dGrid.cellSize, params.bounce);
    int cZ = 0;
    if (!params.is2D) {
        cZ = worldToCellIndexDevice(pos.z, params.worldZ, params.dGrid.numCellsZ, params.dGrid.cellSize, params.bounce);
    }

    // Linear hash
    int hash = flattenIndexDevice(cX, cY, cZ, params.dGrid.numCellsX, params.dGrid.numCellsY);

    // Write out
    dHash[currentBoidIdx] = hash;
    dIndex[currentBoidIdx] = currentBoidIdx;
}

__device__ inline int worldToCellIndexDevice(float pos, float worldSize, int cellCount, float cellSize, bool bounce) {
    if (bounce) {
        int cell = (int)floorf(pos / cellSize);
        if (cell < 0) cell = 0;
        if (cell >= cellCount) cell = cellCount - 1;
        return cell;
    }

    if (pos < 0.0f) {
        pos += worldSize;
    } else if (pos >= worldSize) {
        pos -= worldSize;
    }

    int cell = (int)floorf(pos / cellSize);

    if (cell < 0) {
        cell += cellCount;
    } else if (cell >= cellCount) {
        cell -= cellCount;
    }

    return cell;
}

__device__ inline int flattenIndexDevice(int cX, int cY, int cZ, int numCellsX, int numCellsY) {
    return (cZ * numCellsY + cY) * numCellsX + cX;
}
