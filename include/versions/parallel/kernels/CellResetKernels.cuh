/**
 * \file CellResetKernels.cuh
 * \author Jan Koča
 * \date 05-01-2026
 * \brief CUDA kernels for resetting and initializing spatial grid cell ranges.
 *
 * These kernels:
 *  - clear cell start/end ranges before grid rebuild
 *  - compute contiguous index ranges per cell after sorting boids by hash
 */

#pragma once

#include "versions/parallel/ParallelParameters.cuh"
#include "core/DeviceStructures.hpp"

/**
 * \brief Reset spatial grid cell metadata on the device.
 *
 * For each cell:
 *  - clears begin/end ranges for all boid categories
 *  - stores integer cell coordinates (cx, cy, cz)
 *
 * One thread processes one grid cell.
 *
 * \param[in,out] dGrid Device grid buffer.
 */
__global__ void kernelResetCells(DeviceGrid dGrid);

/**
 * \brief Build contiguous cell index ranges from sorted hash indices.
 *
 * Given boids sorted by cell hash:
 *  - detects first index belonging to a cell
 *  - detects last index belonging to a cell
 *  - writes [start, end) range into cell tables
 *
 * One thread processes one boid index.
 *
 * \param[in] boidCount Number of boids in the buffer.
 * \param[in] dHash Sorted cell hash values.
 * \param[out] dCellStart Output index where the cell range begins.
 * \param[out] dCellEnd Output index one past the end of the range.
 */
__global__ void kernelBuildCellRanges(int boidCount,
                                      int* dHash,
                                      int* dCellStart,
                                      int* dCellEnd);
