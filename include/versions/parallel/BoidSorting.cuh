/**
 * \file BoidSorting.cuh
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Host dispatcher for GPU-side sorting of boids by spatial hash.
 *
 * The sort:
 *  - groups boids belonging to the same spatial grid cell
 *  - produces contiguous index ranges per cell
 *  - preserves mapping via parallel index array
 *
 * Sorting backend:
 *  - padding kernel (fillPadding)
 *  - bitonic sorting network (bitonicSortStepKernel)
 */

#pragma once

/**
 * \brief Sort boids by their computed cell hash on the GPU.
 *
 * The function:
 *  - pads hash/index arrays to the next power-of-two length
 *  - fills padding with sentinel values
 *  - executes a full bitonic sort pass
 *
 * \param[in] boidCount Number of valid boids.
 * \param[in,out] dHash Device pointer to hash buffer.
 * \param[in,out] dIndex Device pointer to boid index buffer.
 * \param[in] blockSize CUDA block size used for kernel launches.
 */
void sortBoidsByHash(int boidCount,
                     int* dHash,
                     int* dIndex,
                     int blockSize);
