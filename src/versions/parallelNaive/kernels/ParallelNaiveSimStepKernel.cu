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
__device__ Vec3 makeWeightedForce(const Vec3& d, float w, float maxForce);
__device__ void repelFromWall(float d, float axisSign, float& accAxis, float maxForce, float visualRange, float multiplier);


__global__ void simulationStepParallelNaiveKernel(ParallelNaiveParameters::GPUParams params) {
    // Compute current boid index
    int currentBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;

    // Out of bounds check
    if (currentBoidIdx >= params.boidCount)
        return;
    
    // Fetch boid type
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

__device__ void resolveBasicBoidBehavior(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    // Get boid data
    DeviceBoids& boids = params.dBoids;

    // Load state
    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 vel = boids.vel[currentBoidIdx];
    Vec3 acc = boids.acc[currentBoidIdx];
    Vec3 target = boids.targetPoint[currentBoidIdx];

    // Define accumulators and counters
    Vec3 personalSpace{0,0,0};
    Vec3 posSum{0,0,0};
    Vec3 velSum{0,0,0};

    int neighborCount = 0;
    int distantNeighborCount = 0;

    // Analyze other basic boids
    for (int otherIdxBasics = 0; otherIdxBasics < boids.basicBoidCount; ++otherIdxBasics) {
        size_t otherIdx = boids.basicBoidIndices[otherIdxBasics];
        
        if (otherIdx == currentBoidIdx)
            continue;

        if (neighborCount >= params.maxNeighborsBasic)
            break;

        // Get other boid data
        Vec3 oPos = boids.pos[otherIdx];
        Vec3 oVel = boids.vel[otherIdx];

        Vec3 distVect = periodicDeltaVec(pos, oPos, params);
        float dist2 = sqrLen(distVect);

        if (dist2 > params.visionRangeBasic2)
            continue;

        ++neighborCount;

        float dist = sqrtf(dist2);
        if (dist < params.eps)
            dist = params.eps;

        // Gather data for behavior rules
        if (dist < params.basicBoidRadius * 2.0f) {
            // Separation
            float invd = 1.0f / dist;
            personalSpace.x -= distVect.x * invd;
            personalSpace.y -= distVect.y * invd;
            personalSpace.z -= distVect.z * invd;
        } else {
            // Cohesion
            posSum.x += distVect.x;
            posSum.y += distVect.y;
            posSum.z += distVect.z;
            ++distantNeighborCount;
        }

        // Alignment
        velSum.x += oVel.x;
        velSum.y += oVel.y;
        velSum.z += oVel.z;
    }

    // Calculate cohesion and alignment forces
    Vec3 cohesionForce{0,0,0};
    Vec3 alignmentForce{0,0,0};

    if (distantNeighborCount > 0) {
        float invN = 1.0f / (float)distantNeighborCount;

        cohesionForce = {
            posSum.x * invN,
            posSum.y * invN,
            posSum.z * invN
        };
    }

    if (neighborCount > 0) {
        float invN = 1.0f / (float)neighborCount;

        Vec3 avgVel = {
            velSum.x * invN,
            velSum.y * invN,
            velSum.z * invN
        };

        alignmentForce = {
            avgVel.x - vel.x,
            avgVel.y - vel.y,
            avgVel.z - vel.z
        };
    }

    // Target attraction
    Vec3 toTarget = periodicDeltaVec(pos, target, params);
    float toTarget2 = sqrLen(toTarget);

    if (toTarget2 < params.eps)
        toTarget2 = params.eps;

    float distanceFactor = toTarget2 / 10.0f / params.maxDistanceBetweenPoints;
    float adjustedTargetWeight = params.targetAttractionWeightBasic * distanceFactor;

    // Cruising force
    Vec3 currentDir = normalize(vel, params.eps);

    Vec3 cruisingVel = {
        currentDir.x * params.cruisingSpeedBasic,
        currentDir.y * params.cruisingSpeedBasic,
        currentDir.z * params.cruisingSpeedBasic
    };

    Vec3 cruisingForce = {
        cruisingVel.x - vel.x,
        cruisingVel.y - vel.y,
        cruisingVel.z - vel.z
    };

    // Normalize force directions
    Vec3 cohesionDir = normalize(cohesionForce, params.eps);
    Vec3 alignmentDir = normalize(alignmentForce, params.eps);
    Vec3 separationDir = normalize(personalSpace, params.eps);
    Vec3 targetDir = normalize(toTarget, params.eps);

    Vec3 cohesionW = makeWeightedForce(cohesionDir, params.cohesionWeightBasic, params.maxForce);
    Vec3 alignmentW = makeWeightedForce(alignmentDir, params.alignmentWeightBasic, params.maxForce);
    Vec3 separationW = makeWeightedForce(separationDir, params.separationWeightBasic, params.maxForce);
    Vec3 targetW = makeWeightedForce(targetDir, adjustedTargetWeight, params.maxForce);
    Vec3 cruisingW = makeWeightedForce(cruisingForce, 0.1f, params.maxForce);

    // Apply to acceleration accumulator
    acc.x += cohesionW.x + alignmentW.x + separationW.x + targetW.x + cruisingW.x;
    acc.y += cohesionW.y + alignmentW.y + separationW.y + targetW.y + cruisingW.y;
    acc.z += cohesionW.z + alignmentW.z + separationW.z + targetW.z + cruisingW.z;

    // Predator avoidance
    Vec3 predAvoid{0,0,0};
    int numPred = 0;

    const size_t* preds = boids.predatorBoidIndices;
    int predCount = boids.predatorBoidCount;

    for (int predIdxPredators = 0; predIdxPredators < predCount; ++predIdxPredators) {
        size_t predIdx = preds[predIdxPredators];

        Vec3 pPos = boids.pos[predIdx];
        Vec3 distVec = periodicDeltaVec(pPos, pos, params);

        float dist = sqrtf(sqrLen(distVec));
        if (dist < params.eps)
            dist = params.eps;

        // Predator chase targeting
        if (dist <= params.visionRangePredator) {
            int& tgtIdx  = boids.targetBoidIdx[predIdx];
            float& tgtDist = boids.targetBoidDistance[predIdx];

            if (tgtIdx == -1 || dist < tgtDist) {
                tgtIdx = currentBoidIdx;
                tgtDist = dist;
            }
        }

        if (dist > params.visionRangeBasic)
            continue;

        ++numPred;

        Vec3 away = normalize(distVec, params.eps);
        float escapeWeight = 5.0f / dist;

        predAvoid.x += away.x * escapeWeight;
        predAvoid.y += away.y * escapeWeight;
        predAvoid.z += away.z * escapeWeight;
    }

    // Panic mode override
    if (numPred > 0) {
        Vec3 escape = normalize(predAvoid, params.eps);
        Vec3 escapeForceW = makeWeightedForce(escape, 1.0f, params.maxForce);

        // If escape force is stronger than current acceleration, override it
        if (sqrLen(escapeForceW) > sqrLen(acc)) {
            acc.x = 0.0f;
            acc.y = 0.0f;
            acc.z = 0.0f;
        }

        acc.x += escapeForceW.x;
        acc.y += escapeForceW.y;
        acc.z += escapeForceW.z;
    }

    // Write back
    boids.acc[currentBoidIdx] = acc;
}

__device__ void resolvePredatorBoidBehavior(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;

    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 vel = boids.vel[currentBoidIdx];
    Vec3 acc = boids.acc[currentBoidIdx];

    int& targetIdx  = boids.targetBoidIdx[currentBoidIdx];
    float& targetDist = boids.targetBoidDistance[currentBoidIdx];

    float& stamina = boids.stamina[currentBoidIdx];
    uint8_t& resting = boids.resting[currentBoidIdx];

    // Maintain cruising speed
    Vec3 velDir = normalize(vel, params.eps);

    Vec3 cruisingVel = {
        velDir.x * params.cruisingSpeedPredator,
        velDir.y * params.cruisingSpeedPredator,
        velDir.z * params.cruisingSpeedPredator
    };

    Vec3 cruisingForce = {
        cruisingVel.x - vel.x,
        cruisingVel.y - vel.y,
        cruisingVel.z - vel.z
    };

    float cruisingForceWeight = 0.5f;
    Vec3 cruisingForceW = {
        cruisingForce.x * (params.maxForce * cruisingForceWeight),
        cruisingForce.y * (params.maxForce * cruisingForceWeight),
        cruisingForce.z * (params.maxForce * cruisingForceWeight)
    };

    acc.x += cruisingForceW.x;
    acc.y += cruisingForceW.y;
    acc.z += cruisingForceW.z;

    // Chase current target
    if (targetIdx != -1 && !resting && stamina > 0.0f) {
        // Chasing
        Vec3 tPos = boids.pos[targetIdx];

        Vec3 toTargetVec = periodicDeltaVec(pos, tPos, params);
        Vec3 toTargetDir = normalize(toTargetVec, params.eps);

        float dist = sqrtf(sqrLen(toTargetVec));
        float dist2 = dist * dist;

        acc.x = toTargetDir.x * dist2;
        acc.y = toTargetDir.y * dist2;
        acc.z = toTargetDir.z * dist2;

        stamina -= params.staminaDrainRatePredator * params.dt;
    } else if (stamina <= 0.0f && !resting) {
        // Exhausted -> enter rest mode
        resting = 1;
        stamina = 0.0f;
    } else if (resting && stamina > params.maxStaminaPredator) {
        // Finished resting
        resting = 0;
        stamina = params.maxStaminaPredator;
    } else if (resting) {
        // Recovering stamina during rest
        stamina += params.staminaRecoveryRatePredator * params.dt;
    }

    // Clear target for next frame
    targetIdx  = -1;
    targetDist = -1.0f;

    // Write back
    boids.acc[currentBoidIdx] = acc;
    boids.vel[currentBoidIdx] = vel;
}

__device__ void resolveRest(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;

    resolveMouseInteraction(params, currentBoidIdx);

    resolveObstacleAndWallAvoidance(params, currentBoidIdx);

    resolveDynamics(params, currentBoidIdx);

    resolveCollisions(params, currentBoidIdx);

    // Enforce 2D constraint if needed
    if (params.is2D) {
        boids.pos[currentBoidIdx].z = 0.0f;
        boids.vel[currentBoidIdx].z = 0.0f;
    }
}

__device__ void resolveMouseInteraction(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;
    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 acc = boids.acc[currentBoidIdx];

    const DeviceInteraction& interaction = params.dInteraction;

    if (interaction.type == static_cast<uint8_t>(InteractionType::Empty))
        return;

    acc.x = 0.0f;
    acc.y = 0.0f;
    acc.z = 0.0f;

    Vec3 diff = periodicDeltaVec(interaction.point, pos, params);

    float dist2 = sqrLen(diff);
    if (dist2 < params.eps)
        dist2 = params.eps;

    // Calculate weight based on distance
    float weight = dist2 / params.maxDistanceBetweenPoints2;
    if (weight < 0.0f)
        weight = 0.0f;

    Vec3 dir = normalize(diff, params.eps);
    Vec3 weightedForce = makeWeightedForce(dir, weight*params.mouseInteractionMultiplier, params.maxForce);

    if (interaction.type == static_cast<uint8_t>(InteractionType::Attract)) {
        acc.x -= weightedForce.x;
        acc.y -= weightedForce.y;
        acc.z -= weightedForce.z;
    } else {
        acc.x += weightedForce.x;
        acc.y += weightedForce.y;
        acc.z += weightedForce.z;
    }

    // Write back
    boids.acc[currentBoidIdx] = acc;
}

__device__ void resolveObstacleAndWallAvoidance(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;

    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 acc = boids.acc[currentBoidIdx];

    BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorRadius;

    float visualRange =
        (type == BoidType::Basic)
            ? params.visionRangeBasic
            : params.visionRangePredator;

    // Obstacle avoidance
    Vec3  obsDirSum{0,0,0};
    float obsWeightSum = 0.0f;
    float obsCount = 0;

    for (int obsIdxObstacles = 0; obsIdxObstacles < boids.obstacleBoidCount; ++obsIdxObstacles) {
        size_t obsIdx = boids.obstacleBoidIndices[obsIdxObstacles];
        const Vec3& oPos = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params);
        diff.z = 0.0f;

        float centerDist = sqrtf(sqrLen(diff));
        float combinedRadius = rBoid + params.obstacleRadius;
        float surfaceDist = centerDist - combinedRadius;

        if (surfaceDist > visualRange)
            continue;

        obsCount += 1.0f;

        Vec3 dir = normalize(diff, params.eps);

        float weight = __expf(-0.1f * surfaceDist);

        obsDirSum.x += dir.x * weight;
        obsDirSum.y += dir.y * weight;
        obsDirSum.z += dir.z * weight;

        obsWeightSum += weight;
    }

    if (obsCount >= 1.0f) {
        Vec3 avgDir = {
            obsDirSum.x,
            obsDirSum.y,
            obsDirSum.z
        };

        float averageWeight = obsWeightSum / obsCount;
        Vec3 avoidDir = normalize(avgDir, params.eps);

        float scale = params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        acc.x += avoidDir.x * scale;
        acc.y += avoidDir.y * scale;
        acc.z += avoidDir.z * scale;
    }

    // Wall repulsion (only if bounce is enabled)
    if (!params.bounce) {
        // Write back
        boids.acc[currentBoidIdx] = acc;
        return;
    }

    // Left / right
    repelFromWall(pos.x - rBoid, 1.0f,  acc.x, params.maxForce, visualRange, params.obstacleAvoidanceMultiplier);
    repelFromWall((params.worldX - rBoid) - pos.x, -1.0f, acc.x, params.maxForce, visualRange, params.obstacleAvoidanceMultiplier);

    // Bottom / top
    repelFromWall(pos.y - rBoid, 1.0f,  acc.y, params.maxForce, visualRange, params.obstacleAvoidanceMultiplier);
    repelFromWall((params.worldY - rBoid) - pos.y, -1.0f, acc.y, params.maxForce, visualRange, params.obstacleAvoidanceMultiplier);
    
    if (!params.is2D) {
        // Front / back
        repelFromWall(pos.z - rBoid, 1.0f,  acc.z, params.maxForce, visualRange, params.obstacleAvoidanceMultiplier);
        repelFromWall((params.worldZ - rBoid) - pos.z, -1.0f, acc.z, params.maxForce, visualRange, params.obstacleAvoidanceMultiplier);
    }

    // Write back
    boids.acc[currentBoidIdx] = acc;
}

__device__ void resolveDynamics(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;

    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 vel = boids.vel[currentBoidIdx];
    Vec3 acc = boids.acc[currentBoidIdx];

    BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    float maxSpeed =
        (type == BoidType::Basic)
            ? params.maxSpeedBasic
            : params.maxSpeedPredator;

    float minSpeed =
        (type == BoidType::Basic)
            ? params.minSpeedBasic
            : params.minSpeedPredator;

    // Drag
    if (params.drag > 0.0f) {
        float invStop = 1.0f / (params.dt * params.numStepsToStopDueToMaxDrag);
        Vec3 stopAcc = {
            -vel.x * invStop,
            -vel.y * invStop,
            -vel.z * invStop
        };

        acc.x += stopAcc.x * params.drag;
        acc.y += stopAcc.y * params.drag;
        acc.z += stopAcc.z * params.drag;
    }

    // Noise
    float accMag = sqrtf(sqrLen(acc));
    Vec3 accDir  = normalize(acc, params.eps);

    // TODO: PLACEHOLDER random vector, replace with proper RNG later
    Vec3 randVec = {
        1.0f,
        1.0f,
        1.0f
    };
    Vec3 randDir = normalize(randVec, params.eps);

    // TODO: PLACEHOLDER noise factor, replace with proper parameter later
    // float n = params.noise;
    float n = 0.0f;
    Vec3 blended = {
        accDir.x * (1.0f - n) + randDir.x * n,
        accDir.y * (1.0f - n) + randDir.y * n,
        accDir.z * (1.0f - n) + randDir.z * n
    };

    // Final acceleration
    Vec3 finalDir = normalize(blended, params.eps);

    acc.x = finalDir.x * accMag;
    acc.y = finalDir.y * accMag;
    acc.z = finalDir.z * accMag;

    vel.x += acc.x * params.dt;
    vel.y += acc.y * params.dt;
    vel.z += acc.z * params.dt;

    float speed2 = sqrLen(vel);
    float speed  = sqrtf(speed2);

    if (speed > maxSpeed) {
        float s = maxSpeed / (speed + params.eps);
        vel.x *= s;
        vel.y *= s;
        vel.z *= s;
    } else if (speed < minSpeed) {
        float s = minSpeed / (speed + params.eps);
        vel.x *= s;
        vel.y *= s;
        vel.z *= s;
    }

    pos.x += vel.x * params.dt;
    pos.y += vel.y * params.dt;
    pos.z += vel.z * params.dt;

    // Write back
    boids.pos[currentBoidIdx] = pos;
    boids.vel[currentBoidIdx] = vel;
    boids.acc[currentBoidIdx] = acc;
}

__device__ void resolveCollisions(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    resolveWallCollisions(params, currentBoidIdx);
    resolveObstacleCollisions(params, currentBoidIdx);
}

__device__ void resolveWallCollisions(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;

    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 vel = boids.vel[currentBoidIdx];

    BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorRadius;

    if (params.bounce) {
        if (pos.x < rBoid) {
            pos.x = rBoid;
            vel.x = -vel.x * params.bounceFactor;
            vel.y =  vel.y * params.bounceFactor;
            vel.z =  vel.z * params.bounceFactor;
        } else if (pos.x > params.worldX - rBoid) {
            pos.x = params.worldX - rBoid;
            vel.x = -vel.x * params.bounceFactor;
            vel.y =  vel.y * params.bounceFactor;
            vel.z =  vel.z * params.bounceFactor;
        }

        if (pos.y < rBoid) {
            pos.y = rBoid;
            vel.y = -vel.y * params.bounceFactor;
            vel.x =  vel.x * params.bounceFactor;
            vel.z =  vel.z * params.bounceFactor;
        } else if (pos.y > params.worldY - rBoid) {
            pos.y = params.worldY - rBoid;
            vel.y = -vel.y * params.bounceFactor;
            vel.x =  vel.x * params.bounceFactor;
            vel.z =  vel.z * params.bounceFactor;
        }

        if (!params.is2D) {
            if (pos.z < rBoid) {
                pos.z = rBoid;
                vel.z = -vel.z * params.bounceFactor;
                vel.x =  vel.x * params.bounceFactor;
                vel.y =  vel.y * params.bounceFactor;
            }
            else if (pos.z > params.worldZ - rBoid) {
                pos.z = params.worldZ - rBoid;
                vel.z = -vel.z * params.bounceFactor;
                vel.x =  vel.x * params.bounceFactor;
                vel.y =  vel.y * params.bounceFactor;
            }
        }
    } else {
        if (pos.x < 0.0f) {        
            pos.x += params.worldX;
        } else if (pos.x >= params.worldX) {
            pos.x -= params.worldX;
        }

        if (pos.y < 0.0f) {        
            pos.y += params.worldY;
        } else if (pos.y >= params.worldY) {
            pos.y -= params.worldY;
        }

        if (!params.is2D) {
            if (pos.z < 0.0f) {        
                pos.z += params.worldZ;
            } else if (pos.z >= params.worldZ) {
                pos.z -= params.worldZ;
            }
        }
    }

    // Write back
    boids.pos[currentBoidIdx] = pos;
    boids.vel[currentBoidIdx] = vel;
}

__device__ void resolveObstacleCollisions(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    DeviceBoids& boids = params.dBoids;

    Vec3 pos = boids.pos[currentBoidIdx];
    Vec3 vel = boids.vel[currentBoidIdx];

    BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorRadius;

    const size_t* obsIndices = boids.obstacleBoidIndices;
    int obsCount = boids.obstacleBoidCount;

    for (int obsIdxObstacles = 0; obsIdxObstacles < obsCount; ++obsIdxObstacles) {
        size_t obsIdx = obsIndices[obsIdxObstacles];
        const Vec3& oPos = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params);
        diff.z = 0.0f;

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.obstacleRadius;
        float dist = sqrtf(dist2) - combinedRadius;

        if (dist < 0.0f) {
            Vec3 dir = normalize(diff, params.eps);
            
            pos.x += dir.x * (-dist + params.eps);
            pos.y += dir.y * (-dist + params.eps);
            pos.z += dir.z * (-dist + params.eps);
            float vDotN = vel.x*dir.x + vel.y*dir.y + vel.z*dir.z;

            Vec3 vReflect = {
                vel.x - 2.0f * vDotN * dir.x,
                vel.y - 2.0f * vDotN * dir.y,
                vel.z - 2.0f * vDotN * dir.z
            };

            vel.x = vReflect.x * params.bounceFactor;
            vel.y = vReflect.y * params.bounceFactor;
            vel.z = vReflect.z * params.bounceFactor;
        }
    }

    // Write back
    boids.pos[currentBoidIdx] = pos;
    boids.vel[currentBoidIdx] = vel;
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

__device__ inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const ParallelNaiveParameters::GPUParams& params) {
    Vec3 distVec = {
        to.x - from.x,
        to.y - from.y,
        params.is2D ? 0.0f : (to.z - from.z)
    };

    if (params.bounce)
        return distVec;
        
    distVec.x = periodicDelta(distVec.x, params.worldX);
    distVec.y = periodicDelta(distVec.y, params.worldY);

    if (!params.is2D)
        distVec.z = periodicDelta(distVec.z, params.worldZ);

    return distVec;
}

__device__ Vec3 makeWeightedForce(const Vec3& d, float w, float maxForce) {
    float k = maxForce * w;
    return Vec3{ d.x * k, d.y * k, d.z * k };
};

__device__ void repelFromWall(float d, float axisSign, float& accAxis, float maxForce, float visualRange, float multiplier) {
    if (d < visualRange) {
        float weight = __expf(-0.3f * d);
        accAxis += axisSign * (maxForce * weight * multiplier);
    }
};
