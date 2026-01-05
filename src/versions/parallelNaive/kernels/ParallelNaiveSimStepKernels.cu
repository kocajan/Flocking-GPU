#include <cmath>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaiveParameters.hpp"


// Forward declarations
__device__ void resolveBasicBoidBehavior(ParallelNaiveParameters::GPUParams& params, int boidIdx);
__device__ void resolvePredatorBoidBehavior(ParallelNaiveParameters::GPUParams& params, int boidIdx);
__device__ void resolveRest(ParallelNaiveParameters::GPUParams& params, int boidIdx, BoidType type);

__device__ void resolveMouseInteraction(ParallelNaiveParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveObstacleAndWallAvoidance(ParallelNaiveParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveDynamics(ParallelNaiveParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveCollisions(ParallelNaiveParameters::GPUParams& params, int i, BoidType type);

__device__ void resolveWallCollisions(ParallelNaiveParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveObstacleCollisions(ParallelNaiveParameters::GPUParams& params, int i, BoidType type);

__device__ inline float sqrLen(const Vec3& v);
__device__ inline Vec3 normalize(const Vec3& v, float eps);
__device__ inline float periodicDelta(float d, float worldSize);
__device__ inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const ParallelNaiveParameters::GPUParams& params);
__device__ inline Vec3 makeWeightedForce(const Vec3& d, float w, float maxForce);
__device__ inline void repelFromWall(float d, float axisSign, float& accAxis, float maxForce, float visualRange, float multiplier);


__global__ void simulationStepParallelNaiveBasicBoidsKernel(ParallelNaiveParameters::GPUParams params) {
    // Compute current boid index
    int currentBasicBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;

    // Out of bounds check
    if (currentBasicBoidIdx >= params.dBoids.basicBoidCount)
        return;

    // Zero acceleration
    params.dBoids.accBasic[currentBasicBoidIdx] = {0,0,0};

    // Resolve behavior
    resolveBasicBoidBehavior(params, currentBasicBoidIdx);

    // Resolve rest of dynamics
    resolveRest(params, currentBasicBoidIdx, BoidType::Basic);
}

__global__ void simulationStepParallelNaivePredatorBoidsKernel(ParallelNaiveParameters::GPUParams params) {
    // Compute current boid index
    int currentPredatorBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;

    // Out of bounds check
    if (currentPredatorBoidIdx >= params.dBoids.predatorBoidCount)
        return;

    // Zero acceleration
    params.dBoids.accPredator[currentPredatorBoidIdx] = {0,0,0};

    // Resolve behavior
    resolvePredatorBoidBehavior(params, currentPredatorBoidIdx);

    // Resolve rest of dynamics
    resolveRest(params, currentPredatorBoidIdx, BoidType::Predator);
}

__device__ void resolveBasicBoidBehavior(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    // Get reference to boids
    DeviceBoids& boids = params.dBoids;

    // Current boid fields
    Vec3 pos = boids.posBasic[currentBoidIdx];
    Vec3 vel = boids.velBasic[currentBoidIdx];
    Vec3 acc = boids.accBasic[currentBoidIdx];
    Vec3 target = boids.targetPointBasic[currentBoidIdx];

    // Initialize accumulators
    Vec3 personalSpace{0,0,0};
    Vec3 posSum{0,0,0};
    Vec3 velSum{0,0,0};
    int neighborCount = 0;
    int distantNeighborCount = 0;

    // Analyze other boids
    for (int otherIdx = 0; otherIdx < boids.basicBoidCount; ++otherIdx) {
        // Break if reached max neighbors
        if (neighborCount >= params.maxNeighborsBasic) break;
            
        // Skip self
        if (otherIdx == currentBoidIdx) continue;

        // Get reference to other boid
        const Vec3 oPos = boids.posBasic[otherIdx];
        const Vec3 oVel = boids.velBasic[otherIdx];

        // Compute distance vector
        Vec3 distVec = periodicDeltaVec(pos, oPos, params);

        // Get squared distance
        float d2 = sqrLen(distVec);

        // Skip if out of visual range
        if (d2 > params.visionRangeBasic2)
            continue;

        // Compute distance and avoid zero-length
        float dist = std::sqrt(d2);
        if (dist < params.eps)
            dist = params.eps;

        // Increment neighbor count
        neighborCount++;

        // Separation - only inside protected range
        if (dist < params.basicBoidRadius * 2.0f) {
            float invd = 1.0f / dist;
            personalSpace.x -= distVec.x * invd;
            personalSpace.y -= distVec.y * invd;
            personalSpace.z -= distVec.z * invd;
        } else if (dist >= params.basicBoidRadius * 2.0f) {
            // Cohesion - full vision range but outside protected range
            posSum.x += distVec.x;
            posSum.y += distVec.y;
            posSum.z += distVec.z;
            distantNeighborCount++;
        }

        // Alignment - full vision range
        velSum.x += oVel.x;
        velSum.y += oVel.y;
        velSum.z += oVel.z;
    }

    Vec3 cohesionForce{0,0,0};
    Vec3 alignmentForce{0,0,0};
    if (distantNeighborCount > 0) {
        float invN = 1.0f / distantNeighborCount;

        // Cohesion - steer toward average delta
        cohesionForce = {
            posSum.x * invN,
            posSum.y * invN,
            posSum.z * invN
        };
    }
    
    if (neighborCount > 0) {
        float invN = 1.0f / neighborCount;

        // Alignment - steer toward average velocity
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

    // Move toward local target
    Vec3 toTarget = periodicDeltaVec(pos, target, params);
    float toTargetDist2 = sqrLen(toTarget);
    
    // Recalculate the targetWeight to include the squared distance to target
    if (toTargetDist2 < params.eps)
        toTargetDist2 = params.eps;
    float distanceFactor = toTargetDist2 / 10.0f / params.maxDistanceBetweenPoints;
    float adjustedTargetWeight = params.targetAttractionWeightBasic * distanceFactor;

    // Create for for cruising in the current velocity with the desired cruising speed
    Vec3 currentSpeedDir = normalize(vel, params.eps);
    Vec3 cruisingVel = {
        currentSpeedDir.x * params.cruisingSpeedBasic,
        currentSpeedDir.y * params.cruisingSpeedBasic,
        currentSpeedDir.z * params.cruisingSpeedBasic
    };
    Vec3 cruisingForce = {
        cruisingVel.x - vel.x,
        cruisingVel.y - vel.y,
        cruisingVel.z - vel.z
    };

    // Get directions from the forces
    Vec3 cohesionDir = normalize(cohesionForce, params.eps);
    Vec3 alignmentDir = normalize(alignmentForce, params.eps);
    Vec3 separationDir = normalize(personalSpace, params.eps);
    Vec3 targetDir = normalize(toTarget, params.eps);

    // Get weighted forces
    Vec3 cohesionForceW = makeWeightedForce(cohesionDir, params.cohesionWeightBasic, params.maxForce);
    Vec3 alignmentForceW = makeWeightedForce(alignmentDir, params.alignmentWeightBasic, params.maxForce);
    Vec3 separationForceW = makeWeightedForce(separationDir, params.separationWeightBasic, params.maxForce);
    Vec3 targetForceW = makeWeightedForce(targetDir, adjustedTargetWeight, params.maxForce);
    Vec3 cruisingForceW = makeWeightedForce(cruisingForce, 0.1f, params.maxForce);

    // Apply the average force to acceleration
    acc.x += cohesionForceW.x + alignmentForceW.x + separationForceW.x + targetForceW.x + cruisingForceW.x;
    acc.y += cohesionForceW.y + alignmentForceW.y + separationForceW.y + targetForceW.y + cruisingForceW.y;
    acc.z += cohesionForceW.z + alignmentForceW.z + separationForceW.z + targetForceW.z + cruisingForceW.z;

    // Predator avoidance — simple random flee away from predators
    Vec3 predAvoidanceDir{0,0,0};
    int numPredators = 0;
    for (int predIdx = 0; predIdx < boids.predatorBoidCount; ++predIdx) {
        // Get reference to predator boid
        Vec3& pPos = boids.posPredator[predIdx];

        // Compute distance vector
        Vec3 distVect = periodicDeltaVec(pPos, pos, params);

        float dist = std::sqrt(sqrLen(distVect));
        if (dist < params.eps)
            dist = params.eps;

        // Save info for predator chasing
        if (dist <= params.visionRangePredator) {
            int& tgtIdx = boids.targetBoidIdxPredator[predIdx];
            float& tgtDist = boids.targetBoidDistancePredator[predIdx];

            if (tgtIdx == -1 || dist < tgtDist) {
                tgtIdx = currentBoidIdx;
                tgtDist = dist;
                boids.targetBoidTypePredator[predIdx] = static_cast<uint8_t>(BoidType::Basic);
            }
        }

        // Skip if out of visual range
        if (dist > params.visionRangeBasic)
            continue;

        // Increment predator count
        numPredators++;

        // Basic direction away from predator
        Vec3 away = normalize(distVect, params.eps);

        // Accumulate avoidance direction
        float escapeWeight = 5.0f / dist;
        predAvoidanceDir.x += away.x * escapeWeight;
        predAvoidanceDir.y += away.y * escapeWeight;
        predAvoidanceDir.z += away.z * escapeWeight;
    }

    // Apply escape impulse
    if (numPredators > 0) {
        // Get escape direction
        Vec3 escape = normalize(predAvoidanceDir, params.eps);
        Vec3 escapeForceW = makeWeightedForce(escape, 2.0f, params.maxForce);

        // If escape force is stronger than current acceleration, override it
        if (sqrLen(escapeForceW) > sqrLen(acc)) {
            acc.x = 0.0f;
            acc.y = 0.0f;
            acc.z = 0.0f;
        }

        // Apply the escape force
        acc.x += escapeForceW.x;
        acc.y += escapeForceW.y;
        acc.z += escapeForceW.z;
    }

    // Write back
    boids.accBasic[currentBoidIdx] = acc;
}

__device__ void resolvePredatorBoidBehavior(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx) {
    // Get boids
    DeviceBoids& boids = params.dBoids;

    // Current predator fields
    Vec3 pos = boids.posPredator[currentBoidIdx];
    Vec3 vel = boids.velPredator[currentBoidIdx];
    Vec3 acc = boids.accPredator[currentBoidIdx];

    int targetIdx = boids.targetBoidIdxPredator[currentBoidIdx];
    float targetDist = boids.targetBoidDistancePredator[currentBoidIdx];
    uint8_t targetType = boids.targetBoidTypePredator[currentBoidIdx];

    float stamina = boids.staminaPredator[currentBoidIdx];
    uint8_t resting = boids.restingPredator[currentBoidIdx];

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

    // Chase the target boid if any
    if (targetIdx != -1 && resting == false && stamina > 0.0f && targetType == static_cast<uint8_t>(BoidType::Basic)) {
        // Chasing
        const Vec3& tPos = boids.posBasic[targetIdx];

        Vec3 toTargetVec = periodicDeltaVec(pos, tPos, params);
        Vec3 toTargetDir = normalize(toTargetVec, params.eps);

        float dist = std::sqrt(sqrLen(toTargetVec));
        float dist2 = dist * dist;
        
        // Apply acceleration toward the target with squared distance scaling
        acc.x = toTargetDir.x * dist2;
        acc.y = toTargetDir.y * dist2;
        acc.z = toTargetDir.z * dist2;

        stamina -= params.staminaDrainRatePredator * params.dt;
    } else if (stamina <= 0.0f && resting == false) {
        // Exhausted -> enter rest mode
        resting = 1;
        stamina = 0.0f;
    } else if (resting == 1 && stamina > params.maxStaminaPredator) {
        // Fully recovered -> exit rest mode
        resting = 0;
        stamina = params.maxStaminaPredator;
    } else if (resting == 1) {
        // Recovering stamina during rest
        stamina += params.staminaRecoveryRatePredator * params.dt;
    }

    // Delete the target info for the next round
    targetIdx = -1;
    targetDist = -1.0f;

    // Write back
    boids.targetBoidIdxPredator[currentBoidIdx] = targetIdx;
    boids.targetBoidDistancePredator[currentBoidIdx] = targetDist;
    boids.staminaPredator[currentBoidIdx] = stamina;
    boids.restingPredator[currentBoidIdx] = resting;
    boids.accPredator[currentBoidIdx] = acc;
}

__device__ void resolveRest(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Resolve mouse interactions
    resolveMouseInteraction(params, currentBoidIdx, type);

    // Resolve obstacle avoidance
    resolveObstacleAndWallAvoidance(params, currentBoidIdx, type);

    // Resolve dynamics
    resolveDynamics(params, currentBoidIdx, type);

    // Resolve wall interactions
    resolveCollisions(params, currentBoidIdx, type);

    if (params.is2D) {
        if (type == BoidType::Basic) {
            params.dBoids.posBasic[currentBoidIdx].z = 0.0f;
            params.dBoids.velBasic[currentBoidIdx].z = 0.0f;
        } else if (type == BoidType::Predator) {
            params.dBoids.posPredator[currentBoidIdx].z = 0.0f;
            params.dBoids.velPredator[currentBoidIdx].z = 0.0f;
        }
    }
}

__device__ void resolveMouseInteraction(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Mouse interaction called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }
    
    // Get boids 
    DeviceBoids& boids = params.dBoids;

    // Current boid fields
    Vec3 pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3 acc = (type == BoidType::Basic) 
            ? boids.accBasic[currentBoidIdx] 
            : boids.accPredator[currentBoidIdx];

    const Interaction& interaction = params.interaction;

    if (interaction.type == InteractionType::Empty)
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

    if (interaction.type == InteractionType::Attract) {
        acc.x -= weightedForce.x;
        acc.y -= weightedForce.y;
        acc.z -= weightedForce.z;
    } else {
        acc.x += weightedForce.x;
        acc.y += weightedForce.y;
        acc.z += weightedForce.z;
    }

    // Write back
    if (type == BoidType::Basic) {
        boids.accBasic[currentBoidIdx] = acc;
    } else {
        boids.accPredator[currentBoidIdx] = acc;
    }
}

__device__ void resolveObstacleAndWallAvoidance(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Obstacle avoidance called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    // Get boids
    DeviceBoids& boids = params.dBoids;

    // Current boid fields
    Vec3 pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3 acc = (type == BoidType::Basic) 
            ? boids.accBasic[currentBoidIdx] 
            : boids.accPredator[currentBoidIdx];

    const float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorBoidRadius;

    const float visualRange =
        (type == BoidType::Basic)
            ? params.visionRangeBasic
            : params.visionRangePredator;

    // Obstacle avoidance
    Vec3  obsDirSum{0,0,0};
    float obsWeightSum = 0.0f;
    float obsCount = 0;

    for (int obsIdx = 0; obsIdx < boids.obstacleBoidCount; ++obsIdx) {
        const Vec3& oPos = boids.posObstacle[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params);
        diff.z = 0.0f;

        float centerDist = sqrtf(sqrLen(diff));
        float combinedRadius = rBoid + params.obstacleBoidRadius;
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
        if (type == BoidType::Basic) {
            boids.accBasic[currentBoidIdx] = acc;
        } else {
            boids.accPredator[currentBoidIdx] = acc;
        }
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
    if (type == BoidType::Basic) {
        boids.accBasic[currentBoidIdx] = acc;
    } else {
        boids.accPredator[currentBoidIdx] = acc;
    }
}

__device__ void resolveDynamics(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Dynamics resolution called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    // Get boids
    DeviceBoids& boids = params.dBoids;

    // Current boid references
    Vec3 pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3 vel = (type == BoidType::Basic) 
            ? boids.velBasic[currentBoidIdx] 
            : boids.velPredator[currentBoidIdx];
    Vec3 acc = (type == BoidType::Basic) 
            ? boids.accBasic[currentBoidIdx] 
            : boids.accPredator[currentBoidIdx];

    const float maxSpeed =
        (type == BoidType::Basic)
            ? params.maxSpeedBasic
            : params.maxSpeedPredator;

    const float minSpeed =
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
    if (type == BoidType::Basic) {
        boids.posBasic[currentBoidIdx] = pos;
        boids.velBasic[currentBoidIdx] = vel;
        boids.accBasic[currentBoidIdx] = acc;
    } else {
        boids.posPredator[currentBoidIdx] = pos;
        boids.velPredator[currentBoidIdx] = vel;
        boids.accPredator[currentBoidIdx] = acc;
    }
}

__device__ void resolveCollisions(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Collision resolution called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    resolveWallCollisions(params, currentBoidIdx, type);
    resolveObstacleCollisions(params, currentBoidIdx, type);
}

__device__ void resolveWallCollisions(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Wall collision called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    // Get boids
    DeviceBoids& boids = params.dBoids;

    // Current boid references
    Vec3 pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3 vel = (type == BoidType::Basic) 
            ? boids.velBasic[currentBoidIdx] 
            : boids.velPredator[currentBoidIdx];

    const float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorBoidRadius;

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
    if (type == BoidType::Basic) {
        boids.posBasic[currentBoidIdx] = pos;
        boids.velBasic[currentBoidIdx] = vel;
    } else {
        boids.posPredator[currentBoidIdx] = pos;
        boids.velPredator[currentBoidIdx] = vel;
    }
}

__device__ void resolveObstacleCollisions(ParallelNaiveParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Obstacle collision called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }
    
    // Get boids
    DeviceBoids& boids = params.dBoids;

    // Current boid fields
    Vec3 pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3 vel = (type == BoidType::Basic) 
            ? boids.velBasic[currentBoidIdx] 
            : boids.velPredator[currentBoidIdx];

    const float rBoid = (type == BoidType::Basic) ? 
        params.basicBoidRadius : params.predatorBoidRadius;

    for (int obsIdx = 0; obsIdx < boids.obstacleBoidCount; ++obsIdx) {
        const Vec3& oPos = boids.posObstacle[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params);
        diff.z = 0.0f;

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.obstacleBoidRadius;
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
    if (type == BoidType::Basic) {
        boids.posBasic[currentBoidIdx] = pos;
        boids.velBasic[currentBoidIdx] = vel;
    } else {
        boids.posPredator[currentBoidIdx] = pos;
        boids.velPredator[currentBoidIdx] = vel;
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
