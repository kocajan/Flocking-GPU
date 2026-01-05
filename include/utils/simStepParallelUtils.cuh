#pragma once

#include <iostream>
#include <cuda_runtime.h>

#include "core/Types.hpp"
#include "versions/parallelNaive/ParallelNaiveParameters.cuh"


#define CHECK_ERROR( error ) ( HandleError( error, __FILE__, __LINE__ ) )

static void HandleError(cudaError_t error, const char* file, int line) { 
    if (error != cudaSuccess) { 
        std::cout << cudaGetErrorString(error) << " in " << file << " at line " << line << std::endl; 
        int w = scanf(" "); 
        exit(EXIT_FAILURE); 
    } 
}

__device__ inline float sqrLen(const Vec3& v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

__device__ inline Vec3 normalize(const Vec3& v, float eps) {
    float l2 = sqrLen(v);
    if (l2 < eps) {
        return {0,0,0};
    }

    float inv = rsqrtf(l2);
    return { v.x*inv, v.y*inv, v.z*inv };
}

__device__ inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

__device__ inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, bool is2D, bool bounce,
                                        float worldX, float worldY, float worldZ) {
    Vec3 distVec = {
        to.x - from.x,
        to.y - from.y,
        is2D ? 0.0f : (to.z - from.z)
    };

    if (bounce)
        return distVec;
        
    distVec.x = periodicDelta(distVec.x, worldX);
    distVec.y = periodicDelta(distVec.y, worldY);

    if (!is2D)
        distVec.z = periodicDelta(distVec.z, worldZ);

    return distVec;
}

__device__ inline Vec3 makeWeightedForce(const Vec3& d, float w, float maxForce) {
    float k = maxForce * w;
    return Vec3{ d.x * k, d.y * k, d.z * k };
};

__device__ inline void repelFromWall(float d, float axisSign, float& accAxis, float maxForce, float visualRange, float multiplier) {
    if (d < visualRange) {
        float weight = __expf(-0.3f * d);
        accAxis += axisSign * (maxForce * weight * multiplier);
    }
};
