/**
 * \file SequentialNaive.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of the sequential-naive flocking simulation.
 *
 * Structure:
 *  - step dispatcher
 *  - per-type simulation loops
 *  - behavior resolution
 *  - interaction handling
 *  - dynamics & collisions
 *
 */

#include "utils/SimStepUtils.hpp"
#include "versions/sequentialNaive/SequentialNaive.hpp"
#include "versions/sequentialNaive/SequentialNaiveParameters.hpp"


void simulationStepSequentialNaiveBasicBoids(SequentialNaiveParameters& params);
void simulationStepSequentialNaivePredatorBoids(SequentialNaiveParameters& params);

void resolveBasicBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx);
void resolvePredatorBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx);
void resolveRest(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);

void resolveMouseInteraction(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);
void resolveObstacleAndWallAvoidance(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);
void resolveDynamics(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);

void resolveCollisions(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);
void resolveWallCollisions(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);
void resolveObstacleCollisions(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type);


void simulationStepSequentialNaive(SequentialNaiveParameters& params) {
    // Process basic boids
    simulationStepSequentialNaiveBasicBoids(params);

    // Process predator boids
    simulationStepSequentialNaivePredatorBoids(params);
}

/**
 * \brief Simulates one timestep for all basic boids in a sequential-naive manner.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 */
void simulationStepSequentialNaiveBasicBoids(SequentialNaiveParameters& params) {
    // Get boids
    Boids& boids = params.boids;

    for (int currentBoidIdx = 0; currentBoidIdx < params.boids.basicBoidCount; ++currentBoidIdx) {
        // Zero acceleration
        boids.accBasic[currentBoidIdx] = {0,0,0};

        // Resolve behavior
        resolveBasicBoidBehavior(params, currentBoidIdx);

        // Resolve rest of dynamics
        resolveRest(params, currentBoidIdx, BoidType::Basic);
    }
}

/**
 * \brief Simulates one timestep for all predator boids in a sequential-naive manner.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 */
void simulationStepSequentialNaivePredatorBoids(SequentialNaiveParameters& params) {
    // Get boids
    Boids& boids = params.boids;

    for (int currentBoidIdx = 0; currentBoidIdx < params.boids.predatorBoidCount; ++currentBoidIdx) {
        // Zero acceleration
        boids.accPredator[currentBoidIdx] = {0,0,0};

        // Resolve behavior
        resolvePredatorBoidBehavior(params, currentBoidIdx);

        // Resolve rest of dynamics
        resolveRest(params, currentBoidIdx, BoidType::Predator);
    }
}

/**
 * \brief Computes flocking behavior forces for a basic boid
 *        (separation, cohesion, alignment, target attraction, predator avoidance).
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed basic boid.
 */
void resolveBasicBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = boids.posBasic[currentBoidIdx];
    Vec3& vel = boids.velBasic[currentBoidIdx];
    Vec3& acc = boids.accBasic[currentBoidIdx];
    Vec3& target = boids.targetPointBasic[currentBoidIdx];

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        float k = params.baseForce * weight;
        return Vec3{ dir.x * k, dir.y * k, dir.z * k };
    };

    // Initialize accumulators
    Vec3 personalSpace{0,0,0};
    Vec3 posSum{0,0,0};
    Vec3 velSum{0,0,0};
    uint64_t neighborCount = 0;
    uint64_t distantNeighborCount = 0;

    // Analyze other boids
    for (int otherIdx = 0; otherIdx < boids.basicBoidCount; ++otherIdx) {
        // Break if reached max neighbors
        if (neighborCount >= params.maxNeighborsBasic) break;
            
        // Skip self
        if (otherIdx == currentBoidIdx) continue;

        // Get reference to other boid
        const Vec3& oPos = boids.posBasic[otherIdx];
        const Vec3& oVel = boids.velBasic[otherIdx];

        // Compute distance vector
        Vec3 distVec = periodicDeltaVec(pos, oPos, params.is2D, params.bounce, 
                                        params.worldX, params.worldY, params.worldZ);

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
    Vec3 toTarget = periodicDeltaVec(pos, target, params.is2D, params.bounce, 
                                     params.worldX, params.worldY, params.worldZ);
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
    Vec3 cohesionForceW = makeWeightedForce(cohesionDir, params.cohesionWeightBasic);
    Vec3 alignmentForceW = makeWeightedForce(alignmentDir, params.alignmentWeightBasic);
    Vec3 separationForceW = makeWeightedForce(separationDir, params.separationWeightBasic);
    Vec3 targetForceW = makeWeightedForce(targetDir, adjustedTargetWeight);
    Vec3 cruisingForceW = makeWeightedForce(cruisingForce, 0.1f);

    // Apply the average force to acceleration
    acc.x += cohesionForceW.x + alignmentForceW.x + separationForceW.x + targetForceW.x + cruisingForceW.x;
    acc.y += cohesionForceW.y + alignmentForceW.y + separationForceW.y + targetForceW.y + cruisingForceW.y;
    acc.z += cohesionForceW.z + alignmentForceW.z + separationForceW.z + targetForceW.z + cruisingForceW.z;

    // Predator avoidance — simple random flee away from predators
    Vec3 predAvoidanceDir{0,0,0};
    int numPredators = 0;
    for (int predIdx = 0; predIdx < params.boids.predatorBoidCount; ++predIdx) {
        // Get reference to predator boid
        Vec3& pPos = boids.posPredator[predIdx];

        // Compute distance vector
        Vec3 distVect = periodicDeltaVec(pPos, pos, params.is2D, params.bounce, 
                                         params.worldX, params.worldY, params.worldZ);

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
                boids.targetBoidTypePredator[predIdx] = BoidType::Basic;
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
        Vec3 escapeForceW = makeWeightedForce(escape, 3.0f);

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
}

/**
 * \brief Computes predator steering behavior including cruising, pursuit,
 *        stamina consumption, and rest-state transitions.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed predator boid.
 */
void resolvePredatorBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current predator fields
    Vec3& pos = boids.posPredator[currentBoidIdx];
    Vec3& vel = boids.velPredator[currentBoidIdx];
    Vec3& acc = boids.accPredator[currentBoidIdx];

    int& targetIdx = boids.targetBoidIdxPredator[currentBoidIdx];
    float& targetDist = boids.targetBoidDistancePredator[currentBoidIdx];
    BoidType& targetType = boids.targetBoidTypePredator[currentBoidIdx];

    float& stamina = boids.staminaPredator[currentBoidIdx];
    uint8_t& resting = boids.restingPredator[currentBoidIdx];

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

    // Chase the target boid if any
    if (targetIdx != -1 && resting == false && stamina > 0.0f && targetType == BoidType::Basic) {
        //Chasing
        const Vec3& tPos = boids.posBasic[targetIdx];

        Vec3 toTargetVec = periodicDeltaVec(pos, tPos, params.is2D, params.bounce, 
                                            params.worldX, params.worldY, params.worldZ);
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
}

/**
 * \brief Applies remaining simulation effects for a boid
 *        (interaction, avoidance, dynamics, and collision handling).
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveRest(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
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
            params.boids.posBasic[currentBoidIdx].z = 0.0f;
            params.boids.velBasic[currentBoidIdx].z = 0.0f;
        } else if (type == BoidType::Predator) {
            params.boids.posPredator[currentBoidIdx].z = 0.0f;
            params.boids.velPredator[currentBoidIdx].z = 0.0f;
        }
    }
}

/**
 * \brief Applies mouse-based attract/repel interaction force to the specified boid.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveMouseInteraction(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Mouse interaction called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }
    
    // Get boids 
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3& acc = (type == BoidType::Basic) 
            ? boids.accBasic[currentBoidIdx] 
            : boids.accPredator[currentBoidIdx];

    // Get interaction
    const Interaction& interaction = params.interaction;

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        float k = params.baseForce * weight * params.mouseInteractionMultiplier;
        return Vec3{ dir.x * k, dir.y * k, dir.z * k };
    };

    if (interaction.type == InteractionType::Empty) {
        return;
    }

    // Zero the so far accumulated acceleration
    acc.x = 0.0f;
    acc.y = 0.0f;
    acc.z = 0.0f;

    Vec3 diff = periodicDeltaVec(interaction.point, pos, params.is2D, params.bounce, 
                                 params.worldX, params.worldY, params.worldZ);

    float dist2 = sqrLen(diff);
    if (dist2 < params.eps)
        dist2 = params.eps;

    // Create weight based on distance
    float weight = dist2 / params.maxDistanceBetweenPoints2;

    // Get normalized direction
    Vec3 dir = normalize(diff, params.eps);

    // Calculate weighted force
    Vec3 weightedForce = makeWeightedForce(dir, weight);

    // Apply force based on interaction type
    if (interaction.type == InteractionType::Attract) {
        acc.x -= weightedForce.x;
        acc.y -= weightedForce.y;
        acc.z -= weightedForce.z;
    } else {
        acc.x += weightedForce.x;
        acc.y += weightedForce.y;
        acc.z += weightedForce.z;
    }
}

/**
 * \brief Applies obstacle avoidance steering and optional wall repulsion to a boid.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveObstacleAndWallAvoidance(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Obstacle avoidance called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    // Get boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3& acc = (type == BoidType::Basic) 
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

    // Resolve obstacle avoidance
    Vec3 obsDirSum{0,0,0};
    float obsWeightSum = 0.0f;
    float obsCount = 0;
    for (int obsIdx = 0; obsIdx < boids.obstacleBoidCount; ++obsIdx) {
        const Vec3& oPos = boids.posObstacle[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params.is2D, params.bounce, 
                                     params.worldX, params.worldY, params.worldZ);

        float centerDist = std::sqrt(sqrLen(diff));
        float combinedRadius = rBoid + params.obstacleBoidRadius;
        float surfaceDist = centerDist - combinedRadius;

        if (surfaceDist > visualRange)
            continue;
        obsCount++;

        Vec3 dir = normalize(diff, params.eps);

        // Proximity weight
        float weight = std::exp(-0.1f * surfaceDist);

        // Accumulate weighted direction
        obsDirSum.x += dir.x * weight;
        obsDirSum.y += dir.y * weight;
        obsDirSum.z += dir.z * weight;

        obsWeightSum += weight;
    }


    if (obsCount >= 1.0) {
        // Calculate average direction
        Vec3 avgDir = {
            obsDirSum.x,
            obsDirSum.y,
            obsDirSum.z
        };

        // Calculate average weight
        float averageWeight = obsWeightSum / obsCount;

        Vec3 avoidDir = normalize(avgDir, params.eps);

        // Apply as a single steering vector
        acc.x += avoidDir.x * params.baseForce * averageWeight * params.obstacleAvoidanceMultiplier;
        acc.y += avoidDir.y * params.baseForce * averageWeight * params.obstacleAvoidanceMultiplier;
        acc.z += avoidDir.z * params.baseForce * averageWeight * params.obstacleAvoidanceMultiplier;
    }

    // If the boid is close to walls, apply repelling force only if bounce is enabled
    if (!params.bounce) {
        return;
    }

    auto repelFromWall = [&](float d, float axisSign, float& accAxis)
    {
        if (d < visualRange) {
            float weight = std::exp(-0.3f * d);
            accAxis += axisSign * (params.baseForce * weight * params.obstacleAvoidanceMultiplier);
        }
    };

    // Left wall
    repelFromWall(pos.x - rBoid, 1.0f, acc.x);

    // Right wall
    repelFromWall((params.worldX - rBoid) - pos.x, -1.0f, acc.x);

    // Bottom wall
    repelFromWall(pos.y - rBoid, 1.0f, acc.y);

    // Top wall
    repelFromWall((params.worldY - rBoid) - pos.y, -1.0f, acc.y);

    if (!params.is2D) {
        // Front wall
        repelFromWall(pos.z - rBoid, 1.0f, acc.z);
        // Back wall
        repelFromWall((params.worldZ - rBoid) - pos.z, -1.0f, acc.z);
    }
}

/**
 * \brief Integrates acceleration, velocity, and position for a boid,
 *        applies noise, drag, and enforces speed limits.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveDynamics(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Dynamics resolution called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    // Get boids
    Boids& boids = params.boids;

    // Current boid references
    Vec3& pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3& vel = (type == BoidType::Basic) 
            ? boids.velBasic[currentBoidIdx] 
            : boids.velPredator[currentBoidIdx];
    Vec3& acc = (type == BoidType::Basic) 
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

    // Acceleration drag (simple linear drag)
    if (params.drag > 0.0f) {
        Vec3 stopAcc = {
            -vel.x / params.dt / params.numStepsToStopDueToMaxDrag,
            -vel.y / params.dt / params.numStepsToStopDueToMaxDrag,
            -vel.z / params.dt / params.numStepsToStopDueToMaxDrag
        };

        // Apply fraction of full stop
        acc.x += stopAcc.x * params.drag;
        acc.y += stopAcc.y * params.drag;
        acc.z += stopAcc.z * params.drag;
    }

    // Current steering magnitude
    float accMagnitude = std::sqrt(sqrLen(acc));
    Vec3 accDir = normalize(acc, params.eps);

    // Random unit vector
    Vec3 randVec = {
        ((float)rand()/RAND_MAX - 0.5f),
        ((float)rand()/RAND_MAX - 0.5f),
        ((float)rand()/RAND_MAX - 0.5f)
    };
    Vec3 randDir = normalize(randVec, params.eps);

    // Blend steering with random direction
    Vec3 blended = {
        accDir.x * (1.0f - params.noise) + randDir.x * params.noise,
        accDir.y * (1.0f - params.noise) + randDir.y * params.noise,
        accDir.z * (1.0f - params.noise) + randDir.z * params.noise
    };

    Vec3 finalDir = normalize(blended, params.eps);

    // Preserve original acceleration magnitude
    acc.x = finalDir.x * accMagnitude;
    acc.y = finalDir.y * accMagnitude;
    acc.z = finalDir.z * accMagnitude;

    // Integrate velocity and position
    vel.x += acc.x * params.dt;
    vel.y += acc.y * params.dt;
    vel.z += acc.z * params.dt;

    // Make sure speed limits are enforced
    float speed = std::sqrt(sqrLen(vel));
    if (speed > maxSpeed) {
        float s = maxSpeed / speed;
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
}

/**
 * \brief Resolves collisions for a boid by dispatching wall and obstacle collision handlers.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveCollisions(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Collision resolution called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    resolveWallCollisions(params, currentBoidIdx, type);
    resolveObstacleCollisions(params, currentBoidIdx, type);
}

/**
 * \brief Resolves interactions with world boundaries using bounce or periodic wrapping.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveWallCollisions(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Wall collision called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }

    // Get boids
    Boids& boids = params.boids;

    // Current boid references
    Vec3& pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3& vel = (type == BoidType::Basic) 
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
            vel.y = vel.y * params.bounceFactor;
            vel.z = vel.z * params.bounceFactor;
        } else if (pos.x > params.worldX - rBoid) { 
            pos.x = params.worldX - rBoid; 
            vel.x = -vel.x * params.bounceFactor; 
            vel.y = vel.y * params.bounceFactor;
            vel.z = vel.z * params.bounceFactor;
        }

        if (pos.y < rBoid) { 
            pos.y = rBoid; 
            vel.y = -vel.y * params.bounceFactor; 
            vel.x = vel.x * params.bounceFactor;
            vel.z = vel.z * params.bounceFactor;
        } else if (pos.y > params.worldY - rBoid) { 
            pos.y = params.worldY - rBoid; 
            vel.y = -vel.y * params.bounceFactor; 
            vel.x = vel.x * params.bounceFactor;
            vel.z = vel.z * params.bounceFactor;
        }

        if (!params.is2D) {
            if (pos.z < rBoid) { 
                pos.z = rBoid; 
                vel.z = -vel.z * params.bounceFactor; 
                vel.x = vel.x * params.bounceFactor; 
                vel.y = vel.y * params.bounceFactor; 
            } else if (pos.z > params.worldZ - rBoid) { 
                pos.z = params.worldZ - rBoid; 
                vel.z = -vel.z * params.bounceFactor; 
                vel.x = vel.x * params.bounceFactor; 
                vel.y = vel.y * params.bounceFactor; 
            }
        }
    } else {
        if (pos.x < 0) {
            pos.x += params.worldX;
        } else if (pos.x >= params.worldX) {
            pos.x -= params.worldX;
        }
        
        if (pos.y < 0) {
            pos.y += params.worldY; 
        } else if (pos.y >= params.worldY) {
            pos.y -= params.worldY;
        }
        
        if (!params.is2D) {
            if (pos.z < 0) {
                pos.z += params.worldZ;
            } else if (pos.z >= params.worldZ) {
                pos.z -= params.worldZ;
            }
        }
    }
}

/**
 * \brief Resolves collisions with obstacle boids using push-out correction and bounce reflection.
 *
 * \param[in,out] params Simulation state and configuration parameters.
 * \param[in]     currentBoidIdx Index of the processed boid.
 * \param[in]     type Type of the processed boid (basic or predator).
 */
void resolveObstacleCollisions(SequentialNaiveParameters& params, int currentBoidIdx, BoidType type) {
    // Check if right type
    if (type != BoidType::Basic && type != BoidType::Predator) {
        printf("Warning: Obstacle collision called for unsupported boid type. (%d)\n", static_cast<int>(type));
        return;
    }
    
    // Get boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = (type == BoidType::Basic) 
            ? boids.posBasic[currentBoidIdx] 
            : boids.posPredator[currentBoidIdx];
    Vec3& vel = (type == BoidType::Basic) 
            ? boids.velBasic[currentBoidIdx] 
            : boids.velPredator[currentBoidIdx];

    const float rBoid = (type == BoidType::Basic) ? 
        params.basicBoidRadius : params.predatorBoidRadius;

    // Check collisions with obstacles
    for (int obsIdx = 0; obsIdx < boids.obstacleBoidCount; ++obsIdx) {
        const Vec3& oPos = boids.posObstacle[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params.is2D, params.bounce, 
                                     params.worldX, params.worldY, params.worldZ);

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.obstacleBoidRadius;
        float dist = std::sqrt(dist2) - combinedRadius;

        if (dist < 0.0f) {
            Vec3 dir = normalize(diff);

            // Push boid out of obstacle
            pos.x += dir.x * (-dist + params.eps);
            pos.y += dir.y * (-dist + params.eps);
            pos.z += dir.z * (-dist + params.eps);

            // Reflection calculation
            float vDotN = vel.x*dir.x + vel.y*dir.y + vel.z*dir.z;

            // Reflected velocity (full bounce)
            Vec3 vReflect = {
                vel.x - 2.0f * vDotN * dir.x,
                vel.y - 2.0f * vDotN * dir.y,
                vel.z - 2.0f * vDotN * dir.z
            };

            // Apply bounce factor
            vel.x = vReflect.x * params.bounceFactor;
            vel.y = vReflect.y * params.bounceFactor;
            vel.z = vReflect.z * params.bounceFactor;
        }
    }
}
