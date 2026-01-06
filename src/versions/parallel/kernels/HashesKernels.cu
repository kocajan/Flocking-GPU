/**
 * \file HashesKernels.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of CUDA kernels for computing spatial hashing of boids.
 *
 * Structure:
 *  - device helpers mirroring SpatialGrid CPU logic
 *  - kernelComputeHashes: world position -> cell index -> hash
 */

#include "versions/parallel/ParallelParameters.cuh"
#include "versions/parallel/kernels/HashesKernels.cuh"


__device__ inline int flattenIndexDevice(int cX, int cY, int cZ, int numCellsX, int numCellsY);
__device__ inline int worldToCellIndexDevice(float pos, float worldSize, int cellCount, float cellSize, bool bounce);


__global__ void kernelComputeHashes(ParallelParameters::GPUParams params, Vec3* dPos, int boidCount, int* dHash, int* dIndex) {
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

/**
 * \brief Convert world coordinate to grid cell index on device.
 *
 * Applies:
 *  - clamping when bounce mode is enabled
 *  - periodic wrapping otherwise
 *
 * \param[in] pos Coordinate along an axis.
 * \param[in] worldSize World extent along that axis.
 * \param[in] cellCount Number of cells along that axis.
 * \param[in] cellSize Size of one grid cell.
 * \param[in] bounce Whether bounce or wrapping behavior is used.
 * \return Integer cell index within valid range.
 */
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

/**
 * \brief Flatten 3D cell coordinates into a linear hash index.
 *
 * \param[in] cX Cell coordinate X.
 * \param[in] cY Cell coordinate Y.
 * \param[in] cZ Cell coordinate Z.
 * \param[in] numCellsX Number of cells along X.
 * \param[in] numCellsY Number of cells along Y.
 * \return Linearized cell hash.
 */
__device__ inline int flattenIndexDevice(int cX, int cY, int cZ, int numCellsX, int numCellsY) {
    return (cZ * numCellsY + cY) * numCellsX + cX;
}
