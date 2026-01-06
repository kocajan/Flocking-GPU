/**
 * \file SortKernels.cuh
 * \author Jan Koča
 * \date 05-01-2026
 * \brief CUDA kernels for padding and in-place bitonic sorting of hash/index pairs.
 *
 * These kernels operate on:
 *  - hash array   (cell hash per boid)
 *  - index array  (boid index associated with hash)
 *
 * The sort produces:
 *  - contiguous boid ranges per spatial cell
 *  - stable ordering needed for cell range construction
 */

#pragma once

#include <cuda_runtime.h>

/**
 * \brief Fill padding region of hash/index buffers with sentinel values.
 *
 * Used when the number of sort elements N is rounded up to a power of two.
 * Elements in range [boidCount, N) are marked invalid and ignored by later steps.
 *
 * \param[out] dHash Hash buffer to pad.
 * \param[out] dIndex Index buffer to pad.
 * \param[in] boidCount Number of valid boids.
 * \param[in] N Total padded length (power of two).
 * \param[in] sentinel Sentinel hash value placed in padding slots.
 */
__global__ void fillPadding(int* dHash,
                            int* dIndex,
                            int boidCount,
                            int N,
                            int sentinel);

/**
 * \brief Execute one stage of the bitonic sort network.
 *
 * Performs pairwise compare–swap on (hash, index) pairs according to the
 * current bitonic merge step parameters (j,k).
 *
 * One thread processes one element.
 *
 * \param[in,out] dHash Hash array to sort.
 * \param[in,out] dIndex Index array kept in sync with hashes.
 * \param[in] j Current bitonic subsequence stride.
 * \param[in] k Current bitonic merge block size.
 * \param[in] N Number of elements in arrays.
 */
__global__ void bitonicSortStepKernel(int* dHash,
                                      int* dIndex,
                                      int j,
                                      int k,
                                      int N);
