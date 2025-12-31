#include <cmath>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaiveParameters.hpp"


// Forward declarations
__device__ void resolveBasicBoidBehavior(ParallelNaiveParameters::GPUParams& params, int boidIdx);
__device__ void resolvePredatorBoidBehavior(ParallelNaiveParameters::GPUParams& params, int boidIdx);
__device__ void resolveRest(ParallelNaiveParameters::GPUParams& params, int boidIdx);

__device__ void resolveMouseInteraction(ParallelNaiveParameters::GPUParams& params, int i);
__device__ void resolveObstacleAndWallAvoidance(ParallelNaiveParameters::GPUParams& params, int i);
__device__ void resolveDynamics(ParallelNaiveParameters::GPUParams& params, int i);
__device__ void resolveCollisions(ParallelNaiveParameters::GPUParams& params, int i);

__device__ void resolveWallCollisions(ParallelNaiveParameters::GPUParams& params, int i);
__device__ void resolveObstacleCollisions(ParallelNaiveParameters::GPUParams& params, int i);

__device__ inline float sqrLen(const Vec3& v);
__device__ inline Vec3 normalize(const Vec3& v, float eps);
__device__ inline float periodicDelta(float d, float worldSize);
__device__ inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const ParallelNaiveParameters::GPUParams& params);
__device__ inline Vec3 makeWeightedForce(const Vec3& d, float w, float maxForce);


__global__ void simulationStepParallelNaiveKernel(ParallelNaiveParameters::GPUParams params) {
    int currentBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (currentBoidIdx >= params.dBoidCount)
        return;
    uint8_t tRaw = params.dBoids.type[currentBoidIdx];
    BoidType type = static_cast<BoidType>(tRaw);

    // Skip obstacles
    if (type == BoidType::Obstacle || type == BoidType::Empty)
        return;

    // Zero acceleration
    params.dBoids.acc[currentBoidIdx] = {0,0,0};

    if (type == BoidType::Basic) {
        resolveBasicBoidBehavior(params, currentBoidIdx);
    } else if (type == BoidType::Predator) {
        resolvePredatorBoidBehavior(params, currentBoidIdx);
    }

    resolveRest(params, currentBoidIdx);
}

__device__ void resolveBasicBoidBehavior(ParallelNaiveParameters::GPUParams& params, int i) {
    DeviceBoids& boids = params.dBoids;

    Vec3 p = boids.pos[i];
    Vec3 v = boids.vel[i];
    Vec3 a = boids.acc[i];
    Vec3 target = boids.targetPoint[i];

    // accumulators
    Vec3 personalSpace{0,0,0};
    Vec3 positionSum{0,0,0};
    Vec3 velocitySum{0,0,0};

    int neighborCount = 0;
    int distantNeighborCount = 0;

    const size_t* indices = boids.basicBoidIndices;
    int count = boids.basicBoidCount;

    //
    // =============================
    // Neighbor scan
    // =============================
    //
    for (int k = 0; k < count; ++k) {
        printf("HERE1\n");
        size_t j = indices[k];
        printf("HERE2\n");
        if (j == i)
            continue;

        if (neighborCount >= (int)params.dMaxNeighborsBasic)
            break;

        Vec3 op = boids.pos[j];
        Vec3 ov = boids.vel[j];
        Vec3 d = periodicDeltaVec(p, op, params);
        float d2 = sqrLen(d);

        if (d2 > params.dVisionRangeBasic2)
            continue;

        float dist = sqrtf(d2);
        if (dist < params.dEps)
            dist = params.dEps;

        ++neighborCount;

        //
        // Separation region
        //
        if (dist < params.dBasicBoidRadius * 2.0f)
        {
            float invd = 1.0f / dist;
            personalSpace.x -= d.x * invd;
            personalSpace.y -= d.y * invd;
            personalSpace.z -= d.z * invd;
        }
        //
        // Cohesion region
        //
        else
        {
            positionSum.x += d.x;
            positionSum.y += d.y;
            positionSum.z += d.z;
            ++distantNeighborCount;
        }

        //
        // Alignment (all neighbors)
        //
        velocitySum.x += ov.x;
        velocitySum.y += ov.y;
        velocitySum.z += ov.z;
    }

    //
    // =============================
    // Cohesion / Alignment
    // =============================
    //
    Vec3 cohesionForce{0,0,0};
    Vec3 alignmentForce{0,0,0};

    if (distantNeighborCount > 0)
    {
        float invN = 1.0f / (float)distantNeighborCount;

        cohesionForce = {
            positionSum.x * invN,
            positionSum.y * invN,
            positionSum.z * invN
        };
    }

    if (neighborCount > 0)
    {
        float invN = 1.0f / (float)neighborCount;

        Vec3 avgVel = {
            velocitySum.x * invN,
            velocitySum.y * invN,
            velocitySum.z * invN
        };

        alignmentForce = {
            avgVel.x - v.x,
            avgVel.y - v.y,
            avgVel.z - v.z
        };
    }

    //
    // =============================
    // Target attraction
    // =============================
    //
    Vec3 toTarget = periodicDeltaVec(p, target, params);
    float toTarget2 = sqrLen(toTarget);

    if (toTarget2 < params.dEps)
        toTarget2 = params.dEps;

    float distanceFactor =
        toTarget2 / 10.0f / params.dMaxDistanceBetweenPoints;
    float adjustedTargetWeight =
        params.dTargetAttractionWeightBasic * distanceFactor;

    //
    // Cruising force
    //
    Vec3 currentDir = normalize(v, params.dEps);

    Vec3 cruisingVel = {
        currentDir.x * params.dCruisingSpeedBasic,
        currentDir.y * params.dCruisingSpeedBasic,
        currentDir.z * params.dCruisingSpeedBasic
    };

    Vec3 cruisingForce = {
        cruisingVel.x - v.x,
        cruisingVel.y - v.y,
        cruisingVel.z - v.z
    };

    //
    // Normalize force directions
    //
    Vec3 cohesionDir   = normalize(cohesionForce, params.dEps);
    Vec3 alignmentDir  = normalize(alignmentForce, params.dEps);
    Vec3 separationDir = normalize(personalSpace, params.dEps);
    Vec3 targetDir     = normalize(toTarget, params.dEps);

    Vec3 cohesionW   = makeWeightedForce(cohesionDir,   params.dCohesionWeightBasic, params.dMaxForce);
    Vec3 alignmentW  = makeWeightedForce(alignmentDir,  params.dAlignmentWeightBasic, params.dMaxForce);
    Vec3 separationW = makeWeightedForce(separationDir, params.dSeparationWeightBasic, params.dMaxForce);
    Vec3 targetW     = makeWeightedForce(targetDir,     adjustedTargetWeight, params.dMaxForce);
    Vec3 cruisingW   = makeWeightedForce(cruisingForce, 0.1f, params.dMaxForce);
    //
    // Apply to acceleration accumulator
    //
    a.x += cohesionW.x + alignmentW.x + separationW.x + targetW.x + cruisingW.x;
    a.y += cohesionW.y + alignmentW.y + separationW.y + targetW.y + cruisingW.y;
    a.z += cohesionW.z + alignmentW.z + separationW.z + targetW.z + cruisingW.z;

    //
    // =============================
    // Predator avoidance
    // =============================
    //
    Vec3 predAvoid{0,0,0};
    int numPred = 0;

    const size_t* preds = boids.predatorBoidIndices;
    int predCount = boids.predatorBoidCount;

    for (int k = 0; k < predCount; ++k)
    {
        size_t j = preds[k];

        Vec3 pp = boids.pos[j];
        Vec3 d = periodicDeltaVec(pp, p, params);

        float dist = sqrtf(sqrLen(d));
        if (dist < params.dEps)
            dist = params.dEps;

        //
        // Predator chase bookkeeping
        //
        if (dist <= params.dVisionRangePredator)
        {
            int& tgtIdx  = boids.targetBoidIdx[j];
            float& tgtDist = boids.targetBoidDistance[j];

            if (tgtIdx == -1 || dist < tgtDist)
            {
                tgtIdx = i;
                tgtDist = dist;
            }
        }

        if (dist > params.dVisionRangeBasic)
            continue;

        ++numPred;

        Vec3 away = normalize(d, params.dEps);
        float invd = 5.0f / dist;

        predAvoid.x += away.x * invd;
        predAvoid.y += away.y * invd;
        predAvoid.z += away.z * invd;
    }

    //
    // Panic mode override
    //
    if (numPred > 0)
    {
        a = {0,0,0};

        Vec3 escape = normalize(predAvoid, params.dEps);
        Vec3 escapeW = makeWeightedForce(escape, 1.0f, params.dMaxForce);

        // always override CPU’s condition
        a = escapeW;
    }

    //
    // write back
    //
    boids.acc[i] = a;
}

__device__
void resolvePredatorBoidBehavior(ParallelNaiveParameters::GPUParams& params,
                                 int i)
{
    DeviceBoids& boids = params.dBoids;

    Vec3 p = boids.pos[i];
    Vec3 v = boids.vel[i];
    Vec3 a = boids.acc[i];

    int&   targetIdx  = boids.targetBoidIdx[i];
    float& targetDist = boids.targetBoidDistance[i];

    float& stamina = boids.stamina[i];
    uint8_t& resting = boids.resting[i];

    //
    // Maintain cruising speed
    //
    Vec3 dir = normalize(v, params.dEps);

    Vec3 cruisingVel = {
        dir.x * params.dCruisingSpeedPredator,
        dir.y * params.dCruisingSpeedPredator,
        dir.z * params.dCruisingSpeedPredator
    };

    Vec3 cruisingForce = {
        cruisingVel.x - v.x,
        cruisingVel.y - v.y,
        cruisingVel.z - v.z
    };

    Vec3 cruisingForceW = {
        cruisingForce.x * (params.dMaxForce * 0.5f),
        cruisingForce.y * (params.dMaxForce * 0.5f),
        cruisingForce.z * (params.dMaxForce * 0.5f)
    };

    a.x += cruisingForceW.x;
    a.y += cruisingForceW.y;
    a.z += cruisingForceW.z;

    //
    // Chase current target
    //
    if (targetIdx != -1 && !resting && stamina > 0.0f)
    {
        Vec3 tp = boids.pos[targetIdx];

        Vec3 toTarget = periodicDeltaVec(p, tp, params);
        Vec3 toTargetN = normalize(toTarget, params.dEps);

        float d = sqrtf(sqrLen(toTarget));
        float d2 = d * d;

        a.x = toTargetN.x * d2;
        a.y = toTargetN.y * d2;
        a.z = toTargetN.z * d2;

        stamina -= params.dStaminaDrainRatePredator * params.dDt;
    }
    //
    // Exhausted → enter rest mode
    //
    else if (stamina <= 0.0f && !resting)
    {
        resting = 1;
        stamina = 0.0f;
    }
    //
    // Fully recovered → exit rest mode
    //
    else if (resting && stamina > params.dMaxStaminaPredator)
    {
        resting = 0;
        stamina = params.dMaxStaminaPredator;
    }
    //
    // Recovering during rest
    //
    else if (resting)
    {
        stamina += params.dStaminaRecoveryRatePredator * params.dDt;
    }

    //
    // Clear target for next frame
    //
    targetIdx  = -1;
    targetDist = -1.0f;

    //
    // write back
    //
    boids.acc[i] = a;
    boids.vel[i] = v;
}

__device__
void resolveRest(ParallelNaiveParameters::GPUParams& params,
                 int i)
{
    DeviceBoids& boids = params.dBoids;

    //
    // Mouse interaction
    //
    resolveMouseInteraction(params, i);

    //
    // Obstacles + wall avoidance
    //
    resolveObstacleAndWallAvoidance(params, i);

    //
    // Dynamics integration
    //
    resolveDynamics(params, i);

    //
    // Collisions
    //
    resolveCollisions(params, i);

    //
    // Enforce 2D mode
    //
    if (params.dIs2D)
    {
        boids.pos[i].z = 0.0f;
        boids.vel[i].z = 0.0f;
    }
}

__device__ void resolveMouseInteraction(ParallelNaiveParameters::GPUParams& params, int i) {
    DeviceBoids& boids = params.dBoids;
    Vec3& p = boids.pos[i];
    Vec3& a = boids.acc[i];

    const DeviceInteraction& inter = params.dInteraction;

    auto makeWeightedForce = [&] (const Vec3& dir, float weight) {
        float s = params.dMaxForce * weight * params.dMouseInteractionMultiplier;
        return Vec3{ dir.x * s, dir.y * s, dir.z * s };
    };

    if (inter.type == static_cast<uint8_t>(InteractionType::Empty))
        return;

    // reset acceleration
    a.x = 0.0f;
    a.y = 0.0f;
    a.z = 0.0f;

    Vec3 diff = periodicDeltaVec(inter.point, p, params);

    float dist2 = sqrLen(diff);
    if (dist2 < params.dEps)
        dist2 = params.dEps;

    float weight = dist2 / params.dMaxDistanceBetweenPoints2;
    if (weight < 0.0f)
        weight = 0.0f;

    Vec3 dir = normalize(diff, params.dEps);
    Vec3 weightedForce = makeWeightedForce(dir, weight);

    if (inter.type == static_cast<uint8_t>(InteractionType::Attract)) {
        a.x -= weightedForce.x;
        a.y -= weightedForce.y;
        a.z -= weightedForce.z;
    } else {
        a.x += weightedForce.x;
        a.y += weightedForce.y;
        a.z += weightedForce.z;
    }
}

__device__ void resolveObstacleAndWallAvoidance(ParallelNaiveParameters::GPUParams& params, int i) {
    DeviceBoids& boids = params.dBoids;

    Vec3& p = boids.pos[i];
    Vec3& a = boids.acc[i];

    // type-dependent params
    BoidType type = static_cast<BoidType>(boids.type[i]);

    float rBoid =
        (type == BoidType::Basic)
            ? params.dBasicBoidRadius
            : params.dPredatorRadius;

    float visualRange =
        (type == BoidType::Basic)
            ? params.dVisionRangeBasic
            : params.dVisionRangePredator;

    // ------------ obstacle avoidance -------------
    Vec3  obstacleDirSum{0,0,0};
    float obstacleWeightSum = 0.0f;
    float obstacleCount     = 0.0f;

    const size_t* obsIndices = boids.obstacleBoidIndices;
    int obsCount             = boids.obstacleBoidCount;

    for (int k = 0; k < obsCount; ++k) {
        size_t obsIdx = obsIndices[k];
        const Vec3& po = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(po, p, params);
        diff.z = 0.0f;

        float centerDist = sqrtf(sqrLen(diff));
        float combinedRadius = rBoid + params.dObstacleRadius;
        float surfaceDist = centerDist - combinedRadius;

        if (surfaceDist > visualRange)
            continue;

        obstacleCount += 1.0f;

        Vec3 dir = normalize(diff, params.dEps);

        float weight = __expf(-0.1f * surfaceDist);

        obstacleDirSum.x += dir.x * weight;
        obstacleDirSum.y += dir.y * weight;
        obstacleDirSum.z += dir.z * weight;

        obstacleWeightSum += weight;
    }

    if (obstacleCount >= 1.0f) {
        Vec3 avgDir = {
            obstacleDirSum.x,
            obstacleDirSum.y,
            obstacleDirSum.z
        };

        float averageWeight = obstacleWeightSum / obstacleCount;
        Vec3 avoidDir = normalize(avgDir, params.dEps);

        float scale = params.dMaxForce * averageWeight * params.dObstacleAvoidanceMultiplier;
        a.x += avoidDir.x * scale;
        a.y += avoidDir.y * scale;
        a.z += avoidDir.z * scale;
    }

    // ------------ wall repulsion (only if bounce) -------------
    if (!params.dBounce)
        return;

    auto repelFromWall = [&] (float d, float axisSign, float& accAxis) {
        if (d < visualRange) {
            float weight = __expf(-0.3f * d);
            accAxis += axisSign * (params.dMaxForce * weight * params.dObstacleAvoidanceMultiplier);
        }
    };

    // left / right
    repelFromWall(p.x - rBoid,                1.0f,  a.x);
    repelFromWall((params.dWorldX - rBoid)-p.x, -1.0f, a.x);

    // bottom / top
    repelFromWall(p.y - rBoid,                1.0f,  a.y);
    repelFromWall((params.dWorldY - rBoid)-p.y, -1.0f, a.y);

    if (!params.dIs2D) {
        // floor / ceiling
        repelFromWall(p.z - rBoid,                1.0f,  a.z);
        repelFromWall((params.dWorldZ - rBoid)-p.z, -1.0f, a.z);
    }
}

__device__ void resolveDynamics(ParallelNaiveParameters::GPUParams& params, int i) {
    DeviceBoids& boids = params.dBoids;

    Vec3& p = boids.pos[i];
    Vec3& v = boids.vel[i];
    Vec3& a = boids.acc[i];

    BoidType type = static_cast<BoidType>(boids.type[i]);

    float maxSpeed =
        (type == BoidType::Basic)
            ? params.dMaxSpeedBasic
            : params.dMaxSpeedPredator;

    float minSpeed =
        (type == BoidType::Basic)
            ? params.dMinSpeedBasic
            : params.dMinSpeedPredator;

    // drag
    if (params.dDrag > 0.0f) {
        float invStop = 1.0f / (params.dDt * params.dNumStepsToStopDueToMaxDrag);
        Vec3 stopAcc = {
            -v.x * invStop,
            -v.y * invStop,
            -v.z * invStop
        };

        a.x += stopAcc.x * params.dDrag;
        a.y += stopAcc.y * params.dDrag;
        a.z += stopAcc.z * params.dDrag;
    }

    // steering magnitude
    float accMag = sqrtf(sqrLen(a));
    Vec3 accDir  = normalize(a, params.dEps);

    // pseudo-random vector in [-0.5,0.5]^3
    Vec3 randVec = {
        1.0f,
        1.0f,
        1.0f
    };
    Vec3 randDir = normalize(randVec, params.dEps);

    // blend steering vs randomness
    // float n = params.noise;
    float n = 0.0f;
    Vec3 blended = {
        accDir.x * (1.0f - n) + randDir.x * n,
        accDir.y * (1.0f - n) + randDir.y * n,
        accDir.z * (1.0f - n) + randDir.z * n
    };

    Vec3 finalDir = normalize(blended, params.dEps);

    a.x = finalDir.x * accMag;
    a.y = finalDir.y * accMag;
    a.z = finalDir.z * accMag;

    // integrate
    v.x += a.x * params.dDt;
    v.y += a.y * params.dDt;
    v.z += a.z * params.dDt;

    float speed2 = sqrLen(v);
    float speed  = sqrtf(speed2);

    if (speed > maxSpeed) {
        float s = maxSpeed / (speed + params.dEps);
        v.x *= s;
        v.y *= s;
        v.z *= s;
    } else if (speed < minSpeed) {
        float s = minSpeed / (speed + params.dEps);
        v.x *= s;
        v.y *= s;
        v.z *= s;
    }

    p.x += v.x * params.dDt;
    p.y += v.y * params.dDt;
    p.z += v.z * params.dDt;
}

__device__ void resolveCollisions(ParallelNaiveParameters::GPUParams& params, int i) {
    resolveWallCollisions(params, i);
    resolveObstacleCollisions(params, i);
}

__device__ void resolveWallCollisions(ParallelNaiveParameters::GPUParams& params, int i) {
    DeviceBoids& boids = params.dBoids;

    Vec3& p = boids.pos[i];
    Vec3& v = boids.vel[i];

    BoidType type = static_cast<BoidType>(boids.type[i]);

    float rBoid =
        (type == BoidType::Basic)
            ? params.dBasicBoidRadius
            : params.dPredatorRadius;

    if (params.dBounce) {
        if (p.x < rBoid) {
            p.x = rBoid;
            v.x = -v.x * params.dBounceFactor;
            v.y =  v.y * params.dBounceFactor;
            v.z =  v.z * params.dBounceFactor;
        }
        else if (p.x > params.dWorldX - rBoid) {
            p.x = params.dWorldX - rBoid;
            v.x = -v.x * params.dBounceFactor;
            v.y =  v.y * params.dBounceFactor;
            v.z =  v.z * params.dBounceFactor;
        }

        if (p.y < rBoid) {
            p.y = rBoid;
            v.y = -v.y * params.dBounceFactor;
            v.x =  v.x * params.dBounceFactor;
            v.z =  v.z * params.dBounceFactor;
        }
        else if (p.y > params.dWorldY - rBoid) {
            p.y = params.dWorldY - rBoid;
            v.y = -v.y * params.dBounceFactor;
            v.x =  v.x * params.dBounceFactor;
            v.z =  v.z * params.dBounceFactor;
        }

        if (!params.dIs2D) {
            if (p.z < rBoid) {
                p.z = rBoid;
                v.z = -v.z * params.dBounceFactor;
                v.x =  v.x * params.dBounceFactor;
                v.y =  v.y * params.dBounceFactor;
            }
            else if (p.z > params.dWorldZ - rBoid) {
                p.z = params.dWorldZ - rBoid;
                v.z = -v.z * params.dBounceFactor;
                v.x =  v.x * params.dBounceFactor;
                v.y =  v.y * params.dBounceFactor;
            }
        }
    } else {
        if (p.x < 0.0f)        p.x += params.dWorldX;
        else if (p.x >= params.dWorldX) p.x -= params.dWorldX;

        if (p.y < 0.0f)        p.y += params.dWorldY;
        else if (p.y >= params.dWorldY) p.y -= params.dWorldY;

        if (!params.dIs2D) {
            if (p.z < 0.0f)         p.z += params.dWorldZ;
            else if (p.z >= params.dWorldZ) p.z -= params.dWorldZ;
        }
    }
}

__device__ void resolveObstacleCollisions(ParallelNaiveParameters::GPUParams& params, int i) {
    DeviceBoids& boids = params.dBoids;

    Vec3& p = boids.pos[i];
    Vec3& v = boids.vel[i];

    BoidType type = static_cast<BoidType>(boids.type[i]);

    float rBoid =
        (type == BoidType::Basic)
            ? params.dBasicBoidRadius
            : params.dPredatorRadius;

    const size_t* obsIndices = boids.obstacleBoidIndices;
    int obsCount             = boids.obstacleBoidCount;

    for (int k = 0; k < obsCount; ++k) {
        size_t obsIdx = obsIndices[k];
        const Vec3& po = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(po, p, params);
        diff.z = 0.0f;

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.dObstacleRadius;
        float dist = sqrtf(dist2) - combinedRadius;

        if (dist < 0.0f) {
            Vec3 n = normalize(diff, params.dEps);

            p.x += n.x * (-dist + params.dEps);
            p.y += n.y * (-dist + params.dEps);
            p.z += n.z * (-dist + params.dEps);
            float vDotN = v.x*n.x + v.y*n.y + v.z*n.z;

            Vec3 vReflect = {
                v.x - 2.0f * vDotN * n.x,
                v.y - 2.0f * vDotN * n.y,
                v.z - 2.0f * vDotN * n.z
            };

            v.x = vReflect.x * params.dBounceFactor;
            v.y = vReflect.y * params.dBounceFactor;
            v.z = vReflect.z * params.dBounceFactor;
        }
    }
}


__device__ inline float sqrLen(const Vec3& v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

__device__ inline Vec3 normalize(const Vec3& v, float eps) {
    float l2 = sqrLen(v);
    if (l2 < eps)
        return {0,0,0};

    float inv = rsqrtf(l2);
    return { v.x*inv, v.y*inv, v.z*inv };
}

__device__ inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

__device__ inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const ParallelNaiveParameters::GPUParams& params) {
    Vec3 d = {
        to.x - from.x,
        to.y - from.y,
        params.dIs2D ? 0.0f : (to.z - from.z)
    };

    if (params.dBounce)
        return d;

    d.x = periodicDelta(d.x, params.dWorldX);
    d.y = periodicDelta(d.y, params.dWorldY);

    if (!params.dIs2D)
        d.z = periodicDelta(d.z, params.dWorldZ);

    return d;
}

__device__ inline Vec3 makeWeightedForce(const Vec3& d, float w, float maxForce) {
    float k = maxForce * w;
    return { d.x * k, d.y * k, d.z * k };
}