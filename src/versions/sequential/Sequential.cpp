#include <cmath>
#include <algorithm>

#include "versions/sequential/Sequential.hpp"
#include "versions/sequential/SequentialParameters.hpp" 
#include "versions/sequential/SpatialGrid.hpp"


void resolveBasicBoidBehavior(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx);
void resolvePredatorBoidBehavior(SequentialParameters& params, int currentBoidIdx);
void resolveRest(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx);

void resolveMouseInteraction(SequentialParameters& params, int currentBoidIdx);
void resolveObstacleAndWallAvoidance(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx);
void resolveDynamics(SequentialParameters& params, int currentBoidIdx);

void resolveCollisions(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx);
void resolveWallCollisions(SequentialParameters& params, int currentBoidIdx);
void resolveObstacleCollisions(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx);

inline float periodicDelta(float d, float worldSize);
inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const SequentialParameters& params);

static inline Vec3 normalize(const Vec3& v, const float eps = 1e-5f);
static inline float sqrLen(const Vec3& v);


void simulationStepSequential(SequentialParameters& params) {
    // Get boids
    Boids& boids = params.boids;

    // Create grid for spatial partitioning
    SpatialGrid grid(params);
    grid.build(params);

    for (int currentBoidIdx = 0; currentBoidIdx < params.boids.allBoidCount; ++currentBoidIdx) {
        // Get current boid type
        uint8_t tRaw = boids.type[currentBoidIdx];
        BoidType type = static_cast<BoidType>(tRaw);

        // Skip obstacles
        if (type == BoidType::Obstacle || type == BoidType::Empty)
            continue;

        // Zero acceleration
        boids.acc[currentBoidIdx] = {0,0,0};

        if (type == BoidType::Basic) {
            resolveBasicBoidBehavior(params, grid, currentBoidIdx);
        } else if (type == BoidType::Predator) {
            resolvePredatorBoidBehavior(params, currentBoidIdx);
        } else {
            // Unknown boid type
            printf("Warning: Unknown boid type encountered in simulation step. (type=%d)\n", static_cast<int>(type));
            continue;
        }
        resolveRest(params, grid, currentBoidIdx);
    }
}

void resolveBasicBoidBehavior(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx) {
    // Get reference to boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& vel = boids.vel[currentBoidIdx];
    Vec3& acc = boids.acc[currentBoidIdx];
    Vec3& target = boids.targetPoint[currentBoidIdx];

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        float k = params.maxForce * weight;
        return Vec3{ dir.x * k, dir.y * k, dir.z * k };
    };

    // Initialize accumulators
    Vec3 personalSpace{0,0,0};
    Vec3 posSum{0,0,0};
    Vec3 velSum{0,0,0};
    uint64_t neighborCount = 0;
    uint64_t distantNeighborCount = 0;

    // Analyze other boids
    const auto& neighbors = grid.getNeighborIndices(params, currentBoidIdx, BoidType::Basic);
    for (int otherIdx : neighbors) {
        // Break if reached max neighbors
        if (neighborCount >= params.maxNeighborsBasic)
            break;
            
        // Skip self
        if (otherIdx == currentBoidIdx) continue;

        // Get reference to other boid
        const Vec3& oPos = boids.pos[otherIdx];
        const Vec3& oVel = boids.vel[otherIdx];

        // Compute distance vector
        Vec3 distVec = periodicDeltaVec(pos, oPos, params);

        // Get squared distance
        float dist2 = sqrLen(distVec);

        // Skip if out of visual range
        if (dist2 > params.visionRangeBasic2)
            continue;

        // Compute distance and avoid zero-length
        float dist = std::sqrt(dist2);
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
    float distanceFactor = toTargetDist2 / 10 / params.maxDistanceBetweenPoints;
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
    const auto& predators = grid.getNeighborIndices(params, currentBoidIdx, BoidType::Predator);
    for (int predIdx : predators) {
        // Get reference to predator boid
        Vec3& pPos = boids.pos[predIdx];

        // Compute distance vector
        Vec3 distVect = periodicDeltaVec(pPos, pos, params);

        float dist = std::sqrt(sqrLen(distVect));
        if (dist < params.eps)
            dist = params.eps;

        // Save info for predator chasing
        if (dist <= params.visionRangePredator) {
            int& tgtIdx  = boids.targetBoidIdx[predIdx];
            float& tgtDist = boids.targetBoidDistance[predIdx];

            if (tgtIdx == -1 || dist < tgtDist) {
                tgtIdx = currentBoidIdx;
                tgtDist = dist;
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
        Vec3 escapeForceW = makeWeightedForce(escape, 1.0f);

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

void resolvePredatorBoidBehavior(SequentialParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current predator fields
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& vel = boids.vel[currentBoidIdx];
    Vec3& acc = boids.acc[currentBoidIdx];

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

    // Chase the target boid if any
    if (targetIdx != -1 && resting == false && stamina > 0.0f) {
        // Chasing
        const Vec3& tPos = boids.pos[targetIdx];

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
        resting = true;
        stamina = 0.0f;
    } else if (resting == true && stamina > params.maxStaminaPredator) {
        // Fully recovered -> exit rest mode
        resting = false;
        stamina = params.maxStaminaPredator;
    } else if (resting == true) {
        // Recovering stamina during rest
        stamina += params.staminaRecoveryRatePredator * params.dt;
    }

    // Delete the target info for the next round
    targetIdx = -1;
    targetDist = -1.0f;
}

void resolveRest(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx) {
    // Resolve mouse interactions
    resolveMouseInteraction(params, currentBoidIdx);

    // Resolve obstacle avoidance
    resolveObstacleAndWallAvoidance(params, grid, currentBoidIdx);

    // Resolve dynamics
    resolveDynamics(params, currentBoidIdx);

    // Resolve wall interactions
    resolveCollisions(params, grid, currentBoidIdx);

    if (params.is2D) {
        params.boids.pos[currentBoidIdx].z = 0.0f;
        params.boids.vel[currentBoidIdx].z = 0.0f;
    }
}

void resolveMouseInteraction(SequentialParameters& params, int currentBoidIdx) {
    // Get boids 
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& acc = boids.acc[currentBoidIdx];

    // Get interaction
    const Interaction& interaction = params.interaction;

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        float k = params.maxForce * weight * params.mouseInteractionMultiplier;
        return Vec3{ dir.x * k, dir.y * k, dir.z * k };
    };

    if (interaction.type == InteractionType::Empty) {
        return;
    }

    // Zero the so far accumulated acceleration
    acc.x = 0.0f;
    acc.y = 0.0f;
    acc.z = 0.0f;

    Vec3 diff = periodicDeltaVec(interaction.point, pos, params);

    float dist2 = sqrLen(diff);
    if (dist2 < params.eps)
        dist2 = params.eps;

    // Create weight based on distance
    float weight = dist2 / params.maxDistanceBetweenPoints2;
    if (weight < 0.0f)
        weight = 0.0f;

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

void resolveObstacleAndWallAvoidance(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& acc = boids.acc[currentBoidIdx];

    // Type-dependent parameters
    const BoidType type =
        static_cast<BoidType>(boids.type[currentBoidIdx]);

    const float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorRadius;

    const float visualRange =
        (type == BoidType::Basic)
            ? params.visionRangeBasic
            : params.visionRangePredator;

    // Resolve obstacle avoidance
    Vec3 obsDirSum{0,0,0};
    float obsWeightSum = 0.0f;
    float obsCount = 0;
    auto obsIdxSet = grid.getNeighborIndices(params, currentBoidIdx, BoidType::Obstacle);
    for (int obsIdx : obsIdxSet) {
        const Vec3& oPos = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params);
        diff.z = 0.0f; // Ignore vertical component for obstacle avoidance

        float centerDist = std::sqrt(sqrLen(diff));
        float combinedRadius = rBoid + params.obstacleRadius;
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
        acc.x += avoidDir.x * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        acc.y += avoidDir.y * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        acc.z += avoidDir.z * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
    }

    // If the boid is close to walls, apply repelling force only if bounce is enabled
    if (!params.bounce) {
        return;
    }

    auto repelFromWall = [&](float d, float axisSign, float& accAxis)
    {
        if (d < visualRange) {
            float weight = std::exp(-0.3f * d);
            accAxis += axisSign * (params.maxForce * weight * params.obstacleAvoidanceMultiplier);
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
        // Floor
        repelFromWall(pos.z - rBoid, 1.0f, acc.z);

        // Ceiling
        repelFromWall((params.worldZ - rBoid) - pos.z, -1.0f, acc.z);
    }
}

void resolveDynamics(SequentialParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid references
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& vel = boids.vel[currentBoidIdx];
    Vec3& acc = boids.acc[currentBoidIdx];

    // Type-dependent speed limits
    const BoidType type =
        static_cast<BoidType>(boids.type[currentBoidIdx]);

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

    // Final normalized direction
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

void resolveCollisions(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx) {
    resolveWallCollisions(params, currentBoidIdx);
    resolveObstacleCollisions(params, grid, currentBoidIdx);
}

void resolveWallCollisions(SequentialParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid references
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& vel = boids.vel[currentBoidIdx];

    // Radius depends on boid type
    const BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    const float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorRadius;

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

void resolveObstacleCollisions(SequentialParameters& params, SpatialGrid& grid, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& pos = boids.pos[currentBoidIdx];
    Vec3& vel = boids.vel[currentBoidIdx];

    // Boid type → pick radius
    const BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    const float rBoid = (type == BoidType::Basic) ? 
        params.basicBoidRadius : params.predatorRadius;

    // Check collisions with obstacles
    auto obsIdxSet = grid.getNeighborIndices(params, currentBoidIdx, BoidType::Obstacle);
    for (int obsIdx : obsIdxSet) {
        const Vec3& oPos = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(oPos, pos, params);
        diff.z = 0.0f; // Ignore vertical component for obstacle collisions

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.obstacleRadius;
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

inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const SequentialParameters& params) {
    // Raw difference (to - from)
    Vec3 distVec{
        to.x - from.x,
        to.y - from.y,
        params.is2D ? 0.0f : (to.z - from.z)
    };

    // In bounce mode use plain Euclidean space
    if (params.bounce) {
        return distVec;
    }

    // In wrapping mode adjust by world size
    distVec.x = periodicDelta(distVec.x, params.worldX);
    distVec.y = periodicDelta(distVec.y, params.worldY);
    
    if (!params.is2D) {
        distVec.z = periodicDelta(distVec.z, params.worldZ);
    }

    return distVec;
}

inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

static inline Vec3 normalize(const Vec3& v, const float eps) {
    float l2 = sqrLen(v);
    if (l2 < eps) {
        return {0.0f, 0.0f, 0.0f};
    }
    
    float inv = 1.0f / std::sqrt(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}

static inline float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
