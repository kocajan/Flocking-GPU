/**
 * \file ParallelSimStepKernels.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of optimized neighborhood-aware simulation kernels.
 *
 * Structure:
 *  - kernel entry points (basic / predator)
 *  - behavior resolution inside local grid neighborhoods
 *  - shared physics utilities and collision handling
 */

#include <cmath>
#include <cuda_runtime.h>

#include "versions/parallel/kernels/ParallelSimStepKernels.cuh"

#include "core/DeviceStructures.hpp"
#include "utils/SimStepParallelUtils.cuh"
#include "versions/parallel/ParallelParameters.cuh"


// Forward declarations
__device__ void resolveBasicBoidBehavior(ParallelParameters::GPUParams& params, int boidIdx);
__device__ void resolvePredatorBoidBehavior(ParallelParameters::GPUParams& params, int boidIdx);
__device__ void resolveRest(ParallelParameters::GPUParams& params, int boidIdx, BoidType type);

__device__ void resolveMouseInteraction(ParallelParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveObstacleAndWallAvoidance(ParallelParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveDynamics(ParallelParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveCollisions(ParallelParameters::GPUParams& params, int i, BoidType type);

__device__ void resolveWallCollisions(ParallelParameters::GPUParams& params, int i, BoidType type);
__device__ void resolveObstacleCollisions(ParallelParameters::GPUParams& params, int i, BoidType type);

__device__ inline int calculateCellHash(int cx, int cy, int cz, const ParallelParameters::GPUParams& params);


__global__ void simulationStepParallelBasicBoidsKernel(ParallelParameters::GPUParams params) {
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

__global__ void simulationStepParallelPredatorBoidsKernel(ParallelParameters::GPUParams params) {
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

/**
 * Resolve flocking behavior for a single basic boid using
 * neighbors from the surrounding grid cells.
 *
 * Gathers:
 *  - separation inside protected radius
 *  - cohesion outside protected radius
 *  - alignment across full vision range
 *  - predator avoidance information
 *
 * Writes acceleration contribution into boid accumulator.
 */
__device__ void resolveBasicBoidBehavior(ParallelParameters::GPUParams& params, int currentBoidIdx) {
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

    // Calculate the current boid's cell hash from the position
    int cX = static_cast<int>(pos.x / params.dGrid.cellSize);
    int cY = static_cast<int>(pos.y / params.dGrid.cellSize);
    int cZ = params.is2D ? 0 : static_cast<int>(pos.z / params.dGrid.cellSize);
    int currentBoidCellHash = calculateCellHash(cX, cY, cZ, params);

    int totalCellInNeighborhood = 3 * 3 * (params.is2D ? 1 : 3);
    int zMin = params.is2D ? 0 : -1;

    // Analyze other basic boids
    for (int neigborhoodCellIdx = 0; neigborhoodCellIdx < totalCellInNeighborhood; ++neigborhoodCellIdx) {
        int dX = (neigborhoodCellIdx % 3) - 1;
        int dY = ((neigborhoodCellIdx / 3) % 3) - 1;
        int dZ = (neigborhoodCellIdx / 9) + zMin;

        int cellHash = calculateCellHash(cX + dX, cY + dY, cZ + dZ, params);

        if (cellHash < 0)
            continue;

        int start = params.dGrid.cellStartBasic[cellHash];
        int end = params.dGrid.cellEndBasic[cellHash];

        if (start == -1)
            continue;

        for (int otherIdxBasics = start; otherIdxBasics < end; ++otherIdxBasics) {
            int otherIdx = params.dGrid.indexBasic[otherIdxBasics];
            
            if (otherIdx == currentBoidIdx)
                continue;

            if (neighborCount >= params.maxNeighborsBasic)
                break;

            // Get reference to other boid
            const Vec3 oPos = boids.posBasic[otherIdx];
            const Vec3 oVel = boids.velBasic[otherIdx];

            Vec3 distVec = periodicDeltaVec(pos, oPos, params.is2D, params.bounce,
                                             params.worldX, params.worldY, params.worldZ);
            float dist2 = sqrLen(distVec);

            if (dist2 > params.visionRangeBasic2)
                continue;

            ++neighborCount;

            float dist = sqrtf(dist2);
            if (dist < params.eps)
                dist = params.eps;

            // Separation
            float invd = 1.0f / dist;
            personalSpace.x -= distVec.x * invd;
            personalSpace.y -= distVec.y * invd;
            personalSpace.z -= distVec.z * invd;
            
            // Cohesion
            if (dist > params.basicBoidRadius * 4.0f) {
                posSum.x += distVec.x;
                posSum.y += distVec.y;
                posSum.z += distVec.z;
                distantNeighborCount++;
            }

            // Alignment
            velSum.x += oVel.x;
            velSum.y += oVel.y;
            velSum.z += oVel.z;
        }
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
    Vec3 toTarget = periodicDeltaVec(pos, target, params.is2D, params.bounce,
                                     params.worldX, params.worldY, params.worldZ);
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

    Vec3 cohesionW = makeWeightedForce(cohesionDir, params.cohesionWeightBasic, params.baseForce);
    Vec3 alignmentW = makeWeightedForce(alignmentDir, params.alignmentWeightBasic, params.baseForce);
    Vec3 separationW = makeWeightedForce(separationDir, params.separationWeightBasic, params.baseForce);
    Vec3 targetW = makeWeightedForce(targetDir, adjustedTargetWeight, params.baseForce);
    Vec3 cruisingW = makeWeightedForce(cruisingForce, 0.1f, params.baseForce);

    // Apply to acceleration accumulator
    acc.x += cohesionW.x + alignmentW.x + separationW.x + targetW.x + cruisingW.x;
    acc.y += cohesionW.y + alignmentW.y + separationW.y + targetW.y + cruisingW.y;
    acc.z += cohesionW.z + alignmentW.z + separationW.z + targetW.z + cruisingW.z;

    // Predator avoidance
    Vec3 predAvoid{0,0,0};
    int numPred = 0;

    // Analyze predators
    for (int neigborhoodCellIdx = 0; neigborhoodCellIdx < totalCellInNeighborhood; ++neigborhoodCellIdx) {
        int dX = (neigborhoodCellIdx % 3) - 1;
        int dY = ((neigborhoodCellIdx / 3) % 3) - 1;
        int dZ = (neigborhoodCellIdx / 9) + zMin;

        int cellHash = calculateCellHash(cX + dX, cY + dY, cZ + dZ, params);

        if (cellHash < 0)
            continue;

        int start = params.dGrid.cellStartPredator[cellHash];
        int end = params.dGrid.cellEndPredator[cellHash];

        if (start == -1)
            continue;

        for (int predIdxPredators = start; predIdxPredators < end; ++predIdxPredators) {
            int predIdx = params.dGrid.indexPredator[predIdxPredators];

            Vec3 pPos = boids.posPredator[predIdx];
            Vec3 distVec = periodicDeltaVec(pPos, pos, params.is2D, params.bounce,
                                            params.worldX, params.worldY, params.worldZ);

            float dist = sqrtf(sqrLen(distVec));
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

            if (dist > params.visionRangeBasic)
                continue;

            ++numPred;

            Vec3 away = normalize(distVec, params.eps);
            float escapeWeight = 5.0f / dist;

            predAvoid.x += away.x * escapeWeight;
            predAvoid.y += away.y * escapeWeight;
            predAvoid.z += away.z * escapeWeight;
        }
    }

    // Panic mode override
    if (numPred > 0) {
        Vec3 escape = normalize(predAvoid, params.eps);
        Vec3 escapeForceW = makeWeightedForce(escape, 3.0f, params.baseForce);

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
    boids.accBasic[currentBoidIdx] = acc;
}

/**
 * Resolve predator behavior and stamina / rest cycle.
 *
 * Responsibilities:
 *  - cruising force toward preferred speed
 *  - chase nearest target boid in range
 *  - stamina drain and recovery
 *  - target reset for next frame
 */
__device__ void resolvePredatorBoidBehavior(ParallelParameters::GPUParams& params, int currentBoidIdx) {
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
        cruisingForce.x * (params.baseForce * cruisingForceWeight),
        cruisingForce.y * (params.baseForce * cruisingForceWeight),
        cruisingForce.z * (params.baseForce * cruisingForceWeight)
    };

    acc.x += cruisingForceW.x;
    acc.y += cruisingForceW.y;
    acc.z += cruisingForceW.z;

    // Chase current target
    if (targetIdx != -1 && !resting && stamina > 0.0f && targetType == static_cast<uint8_t>(BoidType::Basic)) {
        // Chasing
        const Vec3& tPos = boids.posBasic[targetIdx];

        Vec3 toTargetVec = periodicDeltaVec(pos, tPos, params.is2D, params.bounce,
                                            params.worldX, params.worldY, params.worldZ);
        Vec3 toTargetDir = normalize(toTargetVec, params.eps);

        float dist = sqrtf(sqrLen(toTargetVec));
        float dist2 = dist * dist;

        // Apply acceleration toward the target with squared distance scaling
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
    boids.targetBoidIdxPredator[currentBoidIdx] = targetIdx;
    boids.targetBoidDistancePredator[currentBoidIdx] = targetDist;
    boids.staminaPredator[currentBoidIdx] = stamina;
    boids.restingPredator[currentBoidIdx] = resting;
    boids.accPredator[currentBoidIdx] = acc;
}

/**
 * Execute the shared post-behavior pipeline:
 *  - mouse interaction
 *  - obstacle + wall avoidance
 *  - dynamics integration
 *  - wall / obstacle collision handling
 */
__device__ void resolveRest(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
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


/**
 * Apply mouse attraction / repulsion interaction force
 * for a single boid of the given type.
 */
__device__ void resolveMouseInteraction(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
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

    Vec3 diff = periodicDeltaVec(interaction.point, pos, params.is2D, params.bounce,
                                params.worldX, params.worldY, params.worldZ);

    float dist2 = sqrLen(diff);
    if (dist2 < params.eps)
        dist2 = params.eps;

    // Calculate weight based on distance
    float weight = dist2 / params.maxDistanceBetweenPoints2;

    Vec3 dir = normalize(diff, params.eps);
    Vec3 weightedForce = makeWeightedForce(dir, weight * params.mouseInteractionMultiplier, params.baseForce);

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

/**
 * Perform neighborhood-based obstacle avoidance +
 * continuous wall repulsion (when bounce enabled).
 */
__device__ void resolveObstacleAndWallAvoidance(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
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

    // Calculate the current boid's cell hash from the position
    int cX = static_cast<int>(pos.x / params.dGrid.cellSize);
    int cY = static_cast<int>(pos.y / params.dGrid.cellSize);
    int cZ = params.is2D ? 0 : static_cast<int>(pos.z / params.dGrid.cellSize);
    int currentBoidCellHash = calculateCellHash(cX, cY, cZ, params);

    int totalCellInNeighborhood = 3 * 3 * (params.is2D ? 1 : 3);
    int zMin = params.is2D ? 0 : -1;

    // Analyze other basic boids
    for (int neigborhoodCellIdx = 0; neigborhoodCellIdx < totalCellInNeighborhood; ++neigborhoodCellIdx) {
        int dX = (neigborhoodCellIdx % 3) - 1;
        int dY = ((neigborhoodCellIdx / 3) % 3) - 1;
        int dZ = (neigborhoodCellIdx / 9) + zMin;

        int cellHash = calculateCellHash(cX + dX, cY + dY, cZ + dZ, params);

        if (cellHash < 0)
            continue;

        int start = params.dGrid.cellStartObstacle[cellHash];
        int end = params.dGrid.cellEndObstacle[cellHash];

        if (start == -1)
            continue;

        for (int obsIdxObstacles = start; obsIdxObstacles < end; ++obsIdxObstacles) {
            size_t obsIdx = params.dGrid.indexObstacle[obsIdxObstacles];

            const Vec3& oPos = boids.posObstacle[obsIdx];

            Vec3 diff = periodicDeltaVec(oPos, pos, params.is2D, params.bounce,
                                        params.worldX, params.worldY, params.worldZ);

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
    }

    if (obsCount >= 1.0f) {
        Vec3 avgDir = {
            obsDirSum.x,
            obsDirSum.y,
            obsDirSum.z
        };

        float averageWeight = obsWeightSum / obsCount;
        Vec3 avoidDir = normalize(avgDir, params.eps);

        float scale = params.baseForce * averageWeight * params.obstacleAvoidanceMultiplier;
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
    repelFromWall(pos.x - rBoid, 1.0f,  acc.x, params.baseForce, visualRange, params.obstacleAvoidanceMultiplier);
    repelFromWall((params.worldX - rBoid) - pos.x, -1.0f, acc.x, params.baseForce, visualRange, params.obstacleAvoidanceMultiplier);

    // Bottom / top
    repelFromWall(pos.y - rBoid, 1.0f,  acc.y, params.baseForce, visualRange, params.obstacleAvoidanceMultiplier);
    repelFromWall((params.worldY - rBoid) - pos.y, -1.0f, acc.y, params.baseForce, visualRange, params.obstacleAvoidanceMultiplier);
    
    if (!params.is2D) {
        // Front / back
        repelFromWall(pos.z - rBoid, 1.0f,  acc.z, params.baseForce, visualRange, params.obstacleAvoidanceMultiplier);
        repelFromWall((params.worldZ - rBoid) - pos.z, -1.0f, acc.z, params.baseForce, visualRange, params.obstacleAvoidanceMultiplier);
    }

    // Write back
    if (type == BoidType::Basic) {
        boids.accBasic[currentBoidIdx] = acc;
    } else {
        boids.accPredator[currentBoidIdx] = acc;
    }
}

/**
 * Integrate acceleration into velocity and position,
 * including drag, clamping to min/max speed,
 * and optional noise blending.
 */
__device__ void resolveDynamics(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
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

/**
 * Dispatch wall + obstacle collision handling.
 */
__device__ void resolveCollisions(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Collision resolution called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    resolveWallCollisions(params, currentBoidIdx, type);
    resolveObstacleCollisions(params, currentBoidIdx, type);
}

/**
 * Handle world-boundary collisions or wrapping,
 * depending on bounce mode.
 */
__device__ void resolveWallCollisions(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
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

/**
 * Handle collisions against obstacle spheres
 * inside neighboring grid cells.
 */
__device__ void resolveObstacleCollisions(ParallelParameters::GPUParams& params, int currentBoidIdx, BoidType type) {
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

    // Calculate the current boid's cell hash from the position
    int cX = static_cast<int>(pos.x / params.dGrid.cellSize);
    int cY = static_cast<int>(pos.y / params.dGrid.cellSize);
    int cZ = params.is2D ? 0 : static_cast<int>(pos.z / params.dGrid.cellSize);
    int currentBoidCellHash = calculateCellHash(cX, cY, cZ, params);

    int totalCellInNeighborhood = 3 * 3 * (params.is2D ? 1 : 3);
    int zMin = params.is2D ? 0 : -1;

    // Analyze other basic boids
    for (int neigborhoodCellIdx = 0; neigborhoodCellIdx < totalCellInNeighborhood; ++neigborhoodCellIdx) {
        int dX = (neigborhoodCellIdx % 3) - 1;
        int dY = ((neigborhoodCellIdx / 3) % 3) - 1;
        int dZ = (neigborhoodCellIdx / 9) + zMin;

        int cellHash = calculateCellHash(cX + dX, cY + dY, cZ + dZ, params);

        if (cellHash < 0)
            continue;

        int start = params.dGrid.cellStartObstacle[cellHash];
        int end = params.dGrid.cellEndObstacle[cellHash];

        if (start == -1)
            continue;

        for (int obsIdxObstacles = start; obsIdxObstacles < end; ++obsIdxObstacles) {
            size_t obsIdx = params.dGrid.indexObstacle[obsIdxObstacles];

            const Vec3& oPos = boids.posObstacle[obsIdx];

            Vec3 diff = periodicDeltaVec(oPos, pos, params.is2D, params.bounce,
                                         params.worldX, params.worldY, params.worldZ);

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

/**
 * Compute a valid flattened grid hash from cell coordinates,
 * applying either boundary clamping (bounce) or wrap-around.
 */
__device__ inline int calculateCellHash(int cX, int cY, int cZ, const ParallelParameters::GPUParams& params) {
    const DeviceGrid& grid = params.dGrid;

    if (params.bounce) {
        if (cX < 0 || cX >= grid.numCellsX) return -1;
        if (cY < 0 || cY >= grid.numCellsY) return -1;
        if (!params.is2D && (cZ < 0 || cZ >= grid.numCellsZ)) return -1;
    } else {
        if (cX < 0) cX += grid.numCellsX;
        if (cX >= grid.numCellsX) cX -= grid.numCellsX;
        if (cY < 0) cY += grid.numCellsY;
        if (cY >= grid.numCellsY) cY -= grid.numCellsY;

        if (!params.is2D) {
            if (cZ < 0) cZ += grid.numCellsZ;
            if (cZ >= grid.numCellsZ) cZ -= grid.numCellsZ;
        } else {
            cZ = 0;
        }
    }

    // Flatten index
    return (cZ * grid.numCellsY + cY) * grid.numCellsX + cX;
}
