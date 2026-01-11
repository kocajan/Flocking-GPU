/**
 * \file SimStepParallelUtils.cuh
 * \author Jan Koča
 * \date 05-01-2026
 * \brief CUDA utility functions and helpers for vector math and error handling.
 *
 * Provides device-side mathematical helpers used in CUDA kernels,
 * including vector normalization, periodic boundary handling,
 * wall repulsion forces, and CUDA error checking.
 */

#pragma once

#include <iostream>
#include <cuda_runtime.h>

#include "core/Types.hpp"
#include "versions/parallelNaive/ParallelNaiveParameters.cuh"

/**
 * \brief Check CUDA runtime API call for errors.
 *
 * Wraps a CUDA API call and forwards the result to \ref HandleError.
 */
#define CHECK_ERROR(error) (HandleError(error, __FILE__, __LINE__))

/**
 * \brief Handle CUDA runtime errors.
 *
 * Prints the CUDA error string together with source location
 * and terminates the program on failure.
 *
 * \param[in] error CUDA error code.
 * \param[in] file Source file where the error occurred.
 * \param[in] line Line number where the error occurred.
 */
static void HandleError(cudaError_t error, const char* file, int line) {
    if (error != cudaSuccess) {
        std::cout << cudaGetErrorString(error)
                  << " in " << file
                  << " at line " << line
                  << std::endl;
        int w = scanf(" ");
        (void)w;
        exit(EXIT_FAILURE);
    }
}

/**
 * \brief Compute squared Euclidean length of a vector (device-side).
 *
 * \param[in] v Input vector.
 * \return Squared length of the vector.
 */
__device__ inline float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

/**
 * \brief Normalize a vector to unit length (device-side).
 *
 * Uses fast reciprocal square root. If the squared length
 * is below \p eps, a zero vector is returned.
 *
 * \param[in] v Input vector.
 * \param[in] eps Threshold below which the vector is considered zero.
 * \return Normalized vector or zero vector.
 */
__device__ inline Vec3 normalize(const Vec3& v, float eps) {
    float l2 = sqrLen(v);
    if (l2 < eps) {
        return {0.0f, 0.0f, 0.0f};
    }

    float inv = rsqrtf(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}

/**
 * \brief Compute minimal signed distance in a periodic domain (device-side).
 *
 * Adjusts the raw difference \p d so that it lies within
 * \f$[-0.5 \cdot worldSize, 0.5 \cdot worldSize]\f$.
 *
 * \param[in] d Raw coordinate difference.
 * \param[in] worldSize Size of the world in the given dimension.
 * \return Periodicity-corrected signed distance.
 */
__device__ inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

/**
 * \brief Compute displacement vector between two positions in a periodic world.
 *
 * Computes the vector from \p from to \p to. If \p bounce is enabled,
 * no periodic correction is applied. In 2D mode, the Z component is set to zero.
 *
 * \param[in] from Source position.
 * \param[in] to Target position.
 * \param[in] is2D Whether the simulation is constrained to 2D.
 * \param[in] bounce Whether non-periodic (bounce) boundaries are used.
 * \param[in] worldX World size in X dimension.
 * \param[in] worldY World size in Y dimension.
 * \param[in] worldZ World size in Z dimension.
 * \return Displacement vector respecting boundary conditions.
 */
__device__ inline Vec3 periodicDeltaVec(
    const Vec3& from,
    const Vec3& to,
    bool is2D,
    bool bounce,
    float worldX,
    float worldY,
    float worldZ
) {
    Vec3 distVec{
        to.x - from.x,
        to.y - from.y,
        is2D ? 0.0f : (to.z - from.z)
    };

    if (bounce) {
        return distVec;
    }

    distVec.x = periodicDelta(distVec.x, worldX);
    distVec.y = periodicDelta(distVec.y, worldY);

    if (!is2D) {
        distVec.z = periodicDelta(distVec.z, worldZ);
    }

    return distVec;
}

/**
 * \brief Scale a direction vector by a weighted force.
 *
 * \param[in] d Direction vector.
 * \param[in] w Weight factor.
 * \param[in] baseForce Base force magnitude.
 * \return Weighted force vector.
 */
__device__ inline Vec3 makeWeightedForce(const Vec3& d, float w, float baseForce) {
    float k = baseForce * w;
    return { d.x * k, d.y * k, d.z * k };
}

/**
 * \brief Apply repulsive acceleration from a wall along a single axis.
 *
 * If the distance \p d is within \p visualRange, an exponentially
 * decaying repulsive force is applied to \p accAxis.
 *
 * \param[in] d Distance to the wall.
 * \param[in] axisSign Direction of repulsion (+1 or -1).
 * \param[in,out] accAxis Accumulated acceleration along the axis.
 * \param[in] baseForce Base repulsion force.
 * \param[in] visualRange Maximum distance at which repulsion applies.
 * \param[in] multiplier Additional scaling factor.
 */
__device__ inline void repelFromWall(
    float d,
    float axisSign,
    float& accAxis,
    float baseForce,
    float visualRange,
    float multiplier
) {
    if (d < visualRange) {
        float weight = __expf(-0.3f * d);
        accAxis += axisSign * (baseForce * weight * multiplier);
    }
}
