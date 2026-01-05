#pragma once

#include <cmath>
#include "core/Types.hpp"


inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to,
                             bool is2D, bool bounce,
                             float worldX, float worldY, float worldZ) {
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

inline float sqrLen(const Vec3& v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

inline Vec3 normalize(const Vec3& v, float eps = 1e-5f) {
    float l2 = sqrLen(v);
    if (l2 < eps) {
        return {0,0,0};
    }

    float inv = 1.0f / std::sqrt(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}
