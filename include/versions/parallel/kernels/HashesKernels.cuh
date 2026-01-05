/**
 * \file HashesKernels.cuh
 * \author Jan Koča
 * \date 01-05-2026
 * \brief CUDA kernel for computing spatial grid hash indices for boids.
 *
 * The kernel:
 *  - maps boid world positions to cell coordinates
 *  - flattens cell coordinates into linear hash values
 *  - outputs parallel index + hash arrays for sorting
 */

#pragma once

#include "versions/parallel/ParallelParameters.cuh"

/**
 * \brief Compute spatial grid hash values for boid positions.
 *
 * For each boid:
 *  - converts its world-space position to (cx,cy,cz) cell index
 *  - applies bounce or wrapping depending on simulation mode
 *  - flattens cell index into a linear hash
 *  - writes hash + original index for radix sorting
 *
 * One thread processes one boid.
 *
 * \param[in] params GPU simulation parameter block.
 * \param[in] dPos Device array of boid positions.
 * \param[in] boidCount Number of boids to process.
 * \param[in, out] dHash Output hash values (one per boid).
 * \param[in, out] dIndex Output boid indices (identity mapping).
 */
__global__ void kernelComputeHashes(ParallelParameters::GPUParams params,
                                    Vec3* dPos,
                                    int boidCount,
                                    int* dHash,
                                    int* dIndex);
