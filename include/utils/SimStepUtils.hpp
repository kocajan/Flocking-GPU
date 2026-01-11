/**
 * \file SimStepUtils.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Mathematical helpers for periodic worlds and vector operations.
 *
 * Provides utilities for computing periodic distances, vector deltas
 * under periodic boundary conditions, and basic vector math.
 */

#pragma once

#include <cmath>

#include "core/Types.hpp"

/**
 * \brief Compute minimal signed distance in a periodic domain.
 *
 * Adjusts the raw difference \p d so that it lies within
 * the interval \f$[-0.5 \cdot worldSize, 0.5 \cdot worldSize]\f$.
 *
 * \param[in] d Raw coordinate difference.
 * \param[in] worldSize Size of the world in the given dimension.
 * \return Periodicity-corrected signed distance.
 */
inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

/**
 * \brief Compute displacement vector between two positions in a periodic world.
 *
 * Computes the vector pointing from \p from to \p to. If \p bounce is enabled,
 * no periodic correction is applied. In 2D mode, the Z component is forced to zero.
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
inline Vec3 periodicDeltaVec(
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
 * \brief Compute squared Euclidean length of a vector.
 *
 * Avoids the square root operation and is suitable for distance comparisons.
 *
 * \param[in] v Input vector.
 * \return Squared length of the vector.
 */
inline float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

/**
 * \brief Normalize a vector to unit length.
 *
 * If the vector length is below \p eps, a zero vector is returned.
 *
 * \param[in] v Input vector.
 * \param[in] eps Threshold below which the vector is considered zero.
 * \return Normalized vector, or zero vector if length is too small.
 */
inline Vec3 normalize(const Vec3& v, float eps = 1e-5f) {
    float l2 = sqrLen(v);
    if (l2 < eps) {
        return {0.0f, 0.0f, 0.0f};
    }

    float inv = 1.0f / std::sqrt(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}
