#include <cmath>
#include <algorithm>

#include "versions/sequentialNaive/SequentialNaive.hpp"
#include "versions/sequentialNaive/SequentialNaiveParameters.hpp" 


void resolveBasicBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx);
void resolvePredatorBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx);
void resolveRest(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveMouseInteraction(SequentialNaiveParameters& params, int currentBoidIdx);
void resolveObstacleAndWallAvoidance(SequentialNaiveParameters& params, int currentBoidIdx);
void resolveDynamics(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveCollisions(SequentialNaiveParameters& params, int currentBoidIdx);
void resolveWallCollisions(SequentialNaiveParameters& params, int currentBoidIdx);
void resolveObstacleCollisions(SequentialNaiveParameters& params, int currentBoidIdx);

inline float periodicDelta(float d, float worldSize);
inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const SequentialNaiveParameters& params);

static inline Vec3 normalize(const Vec3& v, const float eps = 1e-5f);
static inline float sqrLen(const Vec3& v);


void simulationStepSequentialNaive(SequentialNaiveParameters& params) {
    // Get boids
    Boids& boids = params.boids;

    for (int currentBoidIdx = 0; currentBoidIdx < params.boidCount; ++currentBoidIdx) {
        // Get current boid type
        uint8_t tRaw = boids.type[currentBoidIdx];
        BoidType type = static_cast<BoidType>(tRaw);

        // Skip obstacles
        if (type == BoidType::Obstacle || type == BoidType::Empty)
            continue;

        // Zero acceleration
        boids.acc[currentBoidIdx] = {0,0,0};

        if (type == BoidType::Basic) {
            resolveBasicBoidBehavior(params, currentBoidIdx);
        } else if (type == BoidType::Predator) {
            resolvePredatorBoidBehavior(params, currentBoidIdx);
        } else {
            // Unknown boid type
            printf("Warning: Unknown boid type encountered in simulation step. (type=%d)\n", static_cast<int>(type));
            continue;
        }
        resolveRest(params, currentBoidIdx);
    }
}

void resolveBasicBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& v = boids.vel[currentBoidIdx];
    Vec3& a = boids.acc[currentBoidIdx];
    Vec3& target = boids.targetPoint[currentBoidIdx];

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        return Vec3{
            dir.x * (params.maxForce * weight),
            dir.y * (params.maxForce * weight),
            dir.z * (params.maxForce * weight)
        };
    };

    // Initialize accumulators
    Vec3 personalSpace{0,0,0};
    Vec3 positionSum{0,0,0};
    Vec3 velocitySum{0,0,0};
    uint64_t neighborCount = 0;
    uint64_t distantNeighborCount = 0;

    // Analyze other boids
    for (int otherIdx : params.basicBoidIndices) {
        // Break if reached max neighbors
        if (neighborCount >= params.maxNeighborsBasic)
            break;
            
        // Skip self
        if (otherIdx == currentBoidIdx) continue;

        // Get reference to other boid
        const Vec3& op = boids.pos[otherIdx];
        const Vec3& ov = boids.vel[otherIdx];

        // Compute distance vector
        Vec3 d = periodicDeltaVec(p, op, params);

        // Get squared distance
        float d2 = sqrLen(d);

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
            personalSpace.x -= d.x * invd;
            personalSpace.y -= d.y * invd;
            personalSpace.z -= d.z * invd;
        } else if (dist >= params.basicBoidRadius * 2.0f) {
            // Cohesion - full vision range but outside protected range
            positionSum.x += d.x;
            positionSum.y += d.y;
            positionSum.z += d.z;
            distantNeighborCount++;
        }

        // Alignment - full vision range
        velocitySum.x += ov.x;
        velocitySum.y += ov.y;
        velocitySum.z += ov.z;
    }

    Vec3 cohesionForce{0,0,0};
    Vec3 alignmentForce{0,0,0};
    if (distantNeighborCount > 0) {
        float invN = 1.0f / distantNeighborCount;

        // Cohesion - steer toward average delta
        cohesionForce = {
            positionSum.x * invN,
            positionSum.y * invN,
            positionSum.z * invN
        };
    }
    
    if (neighborCount > 0) {
        float invN = 1.0f / neighborCount;

        // Alignment - steer toward average velocity
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

    // Move toward local target
    Vec3 toTarget = periodicDeltaVec(p, target, params);
    float toTargetDist2 = sqrLen(toTarget);
    
    // Recalculate the targetWeight to include the squared distance to target
    if (toTargetDist2 < params.eps)
        toTargetDist2 = params.eps;
    float distanceFactor = toTargetDist2 / 10 / params.maxDistanceBetweenPoints;
    float adjustedTargetWeight = params.targetAttractionWeightBasic * distanceFactor;

    // Create for for cruising in the current velocity with the desired cruising speed
    Vec3 currentSpeedDir = normalize(v, params.eps);
    Vec3 cruisingVel = {
        currentSpeedDir.x * params.cruisingSpeedBasic,
        currentSpeedDir.y * params.cruisingSpeedBasic,
        currentSpeedDir.z * params.cruisingSpeedBasic
    };
    Vec3 cruisingForce = {
        cruisingVel.x - v.x,
        cruisingVel.y - v.y,
        cruisingVel.z - v.z
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
    a.x += cohesionForceW.x + alignmentForceW.x + separationForceW.x + targetForceW.x + cruisingForceW.x;
    a.y += cohesionForceW.y + alignmentForceW.y + separationForceW.y + targetForceW.y + cruisingForceW.y;
    a.z += cohesionForceW.z + alignmentForceW.z + separationForceW.z + targetForceW.z + cruisingForceW.z;

    // Predator avoidance — simple random flee away from predators
    Vec3 predAvoidanceDir{0,0,0};
    int numPredators = 0;
    for (size_t predIdx : params.predatorBoidIndices) {
        // Get reference to predator boid
        Vec3& pp = boids.pos[predIdx];

        // Compute distance vector
        Vec3 distVect = periodicDeltaVec(pp, p, params);

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
        float invd = 5.0f / dist;
        predAvoidanceDir.x += away.x * invd;
        predAvoidanceDir.y += away.y * invd;
        predAvoidanceDir.z += away.z * invd;
    }

    // Apply escape impulse
    if (numPredators > 0) {
        // Panic - ignore flocking for this frame
        a = {0,0,0};

        // Get escape direction
        Vec3 escape = normalize(predAvoidanceDir, params.eps);
        Vec3 escapeForceW = makeWeightedForce(escape, 1.0f);

        // If escape force is stronger than current acceleration, override it
        if (sqrLen(escapeForceW) > sqrLen(a)) {
            a.x = 0.0f;
            a.y = 0.0f;
            a.z = 0.0f;
        }

        // Apply the escape force
        a.x += escapeForceW.x;
        a.y += escapeForceW.y;
        a.z += escapeForceW.z;
    }
}

void resolvePredatorBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current predator fields
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& v = boids.vel[currentBoidIdx];
    Vec3& a = boids.acc[currentBoidIdx];

    int& targetIdx  = boids.targetBoidIdx[currentBoidIdx];
    float& targetDist = boids.targetBoidDistance[currentBoidIdx];

    float& stamina = boids.stamina[currentBoidIdx];
    uint8_t& resting = boids.resting[currentBoidIdx];

    // Maintain cruising speed
    Vec3 currentSpeedDir = normalize(v, params.eps);
    Vec3 cruisingVel = {
        currentSpeedDir.x * params.cruisingSpeedPredator,
        currentSpeedDir.y * params.cruisingSpeedPredator,
        currentSpeedDir.z * params.cruisingSpeedPredator
    };
    Vec3 cruisingForce = {
        cruisingVel.x - v.x,
        cruisingVel.y - v.y,
        cruisingVel.z - v.z
    };
    Vec3 cruisingForceW = {
        cruisingForce.x * (params.maxForce * 0.5f),
        cruisingForce.y * (params.maxForce * 0.5f),
        cruisingForce.z * (params.maxForce * 0.5f)
    };
    a.x += cruisingForceW.x;
    a.y += cruisingForceW.y;
    a.z += cruisingForceW.z;

    // Chase the target boid if any
    if (targetIdx != -1 && resting == false && stamina > 0.0f) {
        const Vec3& tp = boids.pos[targetIdx];
        Vec3 toTarget = periodicDeltaVec(p, tp, params);
        Vec3 toTargetN = normalize(toTarget, params.eps);
        float toTargetLen = std::sqrt(sqrLen(toTarget));
        a.x = toTargetN.x * (toTargetLen * toTargetLen);
        a.y = toTargetN.y * (toTargetLen * toTargetLen);
        a.z = toTargetN.z * (toTargetLen * toTargetLen);
        stamina -= params.staminaDrainRatePredator * params.dt;
    } else if (stamina <= 0.0f && resting == false) {
        resting = true;
        stamina = 0.0f;
    } else if (resting == true && stamina > params.maxStaminaPredator) {
        resting = false;
        stamina = params.maxStaminaPredator;
    } else if (resting == true) {
        stamina += params.staminaRecoveryRatePredator * params.dt;
    }

    // Delete the target info for the next round
    targetIdx = -1;
    targetDist = -1.0f;
}

void resolveRest(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Resolve mouse interactions
    resolveMouseInteraction(params, currentBoidIdx);

    // Resolve obstacle avoidance
    resolveObstacleAndWallAvoidance(params, currentBoidIdx);

    // Resolve dynamics
    resolveDynamics(params, currentBoidIdx);

    // Resolve wall interactions
    resolveCollisions(params, currentBoidIdx);

    if (params.is2D) {
        params.boids.pos[currentBoidIdx].z = 0.0f;
        params.boids.vel[currentBoidIdx].z = 0.0f;
    }
}

void resolveMouseInteraction(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids 
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& a = boids.acc[currentBoidIdx];

    // Get interaction
    const Interaction& inter = params.interaction;

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        return Vec3{
            dir.x * (params.maxForce * weight) * params.mouseInteractionMultiplier,
            dir.y * (params.maxForce * weight) * params.mouseInteractionMultiplier,
            dir.z * (params.maxForce * weight) * params.mouseInteractionMultiplier
        };
    };

    if (inter.type != InteractionType::Empty) {
        // Zero the so far accumulated acceleration
        a.x = 0.0f;
        a.y = 0.0f;
        a.z = 0.0f;

        Vec3 diff = periodicDeltaVec(inter.point, p, params);

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

        if (inter.type == InteractionType::Attract) {
            a.x -= weightedForce.x;
            a.y -= weightedForce.y;
            a.z -= weightedForce.z;
        } else {
            a.x += weightedForce.x;
            a.y += weightedForce.y;
            a.z += weightedForce.z;
        }
    }
}

void resolveObstacleAndWallAvoidance(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& a = boids.acc[currentBoidIdx];

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
    Vec3 obstacleDirSum{0,0,0};
    float obstacleWeightSum = 0.0f;
    float obstacleCount = 0;
    for (size_t obsIdx : params.obstacleBoidIndices) {
        const Vec3& po = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(po, p, params);
        diff.z = 0.0f; // Ignore vertical component for obstacle avoidance

        float centerDist = std::sqrt(sqrLen(diff));
        float combinedRadius = rBoid + params.obstacleRadius;
        float surfaceDist = centerDist - combinedRadius;

        if (surfaceDist > visualRange)
            continue;
        obstacleCount++;

        Vec3 dir = normalize(diff, params.eps);

        // Proximity weight
        float weight = std::exp(-0.1f * surfaceDist);

        // Accumulate weighted direction
        obstacleDirSum.x += dir.x * weight;
        obstacleDirSum.y += dir.y * weight;
        obstacleDirSum.z += dir.z * weight;

        obstacleWeightSum += weight;
    }


    if (obstacleCount >= 1.0) {
        // Calculate average direction
        Vec3 avgDir = {
            obstacleDirSum.x,
            obstacleDirSum.y,
            obstacleDirSum.z
        };

        // Calculate average weight
        float averageWeight = obstacleWeightSum / obstacleCount;

        Vec3 avoidDir = normalize(avgDir, params.eps);

        // Apply as a single steering vector
        a.x += avoidDir.x * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        a.y += avoidDir.y * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        a.z += avoidDir.z * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
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
    repelFromWall(p.x - rBoid, 1.0f, a.x);

    // Right wall
    repelFromWall((params.worldX - rBoid) - p.x, -1.0f, a.x);

    // Bottom wall
    repelFromWall(p.y - rBoid, 1.0f, a.y);

    // Top wall
    repelFromWall((params.worldY - rBoid) - p.y, -1.0f, a.y);

    if (!params.is2D)
    {
        // Floor
        repelFromWall(p.z - rBoid, 1.0f, a.z);

        // Ceiling
        repelFromWall((params.worldZ - rBoid) - p.z, -1.0f, a.z);
    }
}

void resolveDynamics(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid references
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& v = boids.vel[currentBoidIdx];
    Vec3& a = boids.acc[currentBoidIdx];

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
            -v.x / params.dt / params.numStepsToStopDueToMaxDrag,
            -v.y / params.dt / params.numStepsToStopDueToMaxDrag,
            -v.z / params.dt / params.numStepsToStopDueToMaxDrag
        };

        // apply fraction of full stop
        a.x += stopAcc.x * params.drag;
        a.y += stopAcc.y * params.drag;
        a.z += stopAcc.z * params.drag;
    }

    // Current steering magnitude
    float accMagnitude = std::sqrt(sqrLen(a));
    Vec3 accDir = normalize(a, params.eps);

    // Random unit vector
    Vec3 randVec = {
        ((float)rand()/RAND_MAX - 0.5f),
        ((float)rand()/RAND_MAX - 0.5f),
        ((float)rand()/RAND_MAX - 0.5f)
    };
    Vec3 randDir = normalize(randVec, params.eps);

    // blend steering vs randomness
    Vec3 blended = {
        accDir.x * (1.0f - params.noise) + randDir.x * params.noise,
        accDir.y * (1.0f - params.noise) + randDir.y * params.noise,
        accDir.z * (1.0f - params.noise) + randDir.z * params.noise
    };

    Vec3 finalDir = normalize(blended, params.eps);

    // preserve magnitude
    a.x = finalDir.x * accMagnitude;
    a.y = finalDir.y * accMagnitude;
    a.z = finalDir.z * accMagnitude;

    // Integrate
    v.x += a.x * params.dt;
    v.y += a.y * params.dt;
    v.z += a.z * params.dt;

    // Speed limits (Boids Lab style)
    float speed = std::sqrt(sqrLen(v));
    if (speed > maxSpeed) {
        float s = maxSpeed / speed;
        v.x *= s; 
        v.y *= s; 
        v.z *= s;
    } else if (speed < minSpeed) {
        float s = minSpeed / (speed + params.eps);
        v.x *= s; 
        v.y *= s; 
        v.z *= s;
    }

    p.x += v.x * params.dt;
    p.y += v.y * params.dt;
    p.z += v.z * params.dt;
}

void resolveCollisions(SequentialNaiveParameters& params, int currentBoidIdx) {
    resolveWallCollisions(params, currentBoidIdx);
    resolveObstacleCollisions(params, currentBoidIdx);
}

void resolveWallCollisions(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid references
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& v = boids.vel[currentBoidIdx];

    // Radius depends on boid type
    const BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    const float rBoid =
        (type == BoidType::Basic)
            ? params.basicBoidRadius
            : params.predatorRadius;

    if (params.bounce) {
        if (p.x < rBoid) { 
            p.x = rBoid; 
            v.x = -v.x * params.bounceFactor;
            v.y = v.y * params.bounceFactor;
            v.z = v.z * params.bounceFactor;
        } else if (p.x > params.worldX - rBoid) { 
            p.x = params.worldX - rBoid; 
            v.x = -v.x * params.bounceFactor; 
            v.y = v.y * params.bounceFactor;
            v.z = v.z * params.bounceFactor;
        }

        if (p.y < rBoid) { 
            p.y = rBoid; 
            v.y = -v.y * params.bounceFactor; 
            v.x = v.x * params.bounceFactor;
            v.z = v.z * params.bounceFactor;
        } else if (p.y > params.worldY - rBoid) { 
            p.y = params.worldY - rBoid; 
            v.y = -v.y * params.bounceFactor; 
            v.x = v.x * params.bounceFactor;
            v.z = v.z * params.bounceFactor;
        }

        if (!params.is2D) {
            if (p.z < rBoid) { 
                p.z = rBoid; 
                v.z = -v.z * params.bounceFactor; 
                v.x = v.x * params.bounceFactor; 
                v.y = v.y * params.bounceFactor; 
            } else if (p.z > params.worldZ - rBoid) { 
                p.z = params.worldZ - rBoid; 
                v.z = -v.z * params.bounceFactor; 
                v.x = v.x * params.bounceFactor; 
                v.y = v.y * params.bounceFactor; 
            }
        }
    } else {
        if (p.x < 0) {
            p.x += params.worldX;
        } else if (p.x >= params.worldX) {
            p.x -= params.worldX;
        }
        
        if (p.y < 0) {
            p.y += params.worldY; 
        } else if (p.y >= params.worldY) {
            p.y -= params.worldY;
        }
        
        if (!params.is2D) {
            if (p.z < 0) {
                p.z += params.worldZ;
            } else if (p.z >= params.worldZ) {
                p.z -= params.worldZ;
            }
        }
    }
}

void resolveObstacleCollisions(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get boids
    Boids& boids = params.boids;

    // Current boid fields
    Vec3& p = boids.pos[currentBoidIdx];
    Vec3& v = boids.vel[currentBoidIdx];

    // Boid type → pick radius
    const BoidType type = static_cast<BoidType>(boids.type[currentBoidIdx]);

    const float rBoid = (type == BoidType::Basic) ? 
        params.basicBoidRadius : params.predatorRadius;

    // Check collisions with obstacles
    for (size_t obsIdx : params.obstacleBoidIndices) {
        const Vec3& po = boids.pos[obsIdx];

        Vec3 diff = periodicDeltaVec(po, p, params);
        diff.z = 0.0f; // Ignore vertical component for obstacle collisions

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.obstacleRadius;
        float dist = std::sqrt(dist2) - combinedRadius;

        if (dist < 0.0f) {
            Vec3 n = normalize(diff);

            // Push boid out of obstacle
            p.x += n.x * (-dist + params.eps);
            p.y += n.y * (-dist + params.eps);
            p.z += n.z * (-dist + params.eps);

            // Reflection calculation
            float vDotN = v.x*n.x + v.y*n.y + v.z*n.z;

            // Reflected velocity (full bounce)
            Vec3 vReflect = {
                v.x - 2.0f * vDotN * n.x,
                v.y - 2.0f * vDotN * n.y,
                v.z - 2.0f * vDotN * n.z
            };

            // Apply bounce factor
            v.x = vReflect.x * params.bounceFactor;
            v.y = vReflect.y * params.bounceFactor;
            v.z = vReflect.z * params.bounceFactor;
        }
    }
}


inline Vec3 periodicDeltaVec(const Vec3& from, const Vec3& to, const SequentialNaiveParameters& params) {
    // Raw difference (to - from)
    Vec3 d{
        to.x - from.x,
        to.y - from.y,
        params.is2D ? 0.0f : (to.z - from.z)
    };

    // In bounce mode use plain Euclidean space
    if (params.bounce)
        return d;

    // In wrapping mode adjust by world size
    d.x = periodicDelta(d.x, params.worldX);
    d.y = periodicDelta(d.y, params.worldY);

    if (!params.is2D) {
        d.z = periodicDelta(d.z, params.worldZ);
    }

    return d;
}

inline float periodicDelta(float d, float worldSize) {
    if (d >  0.5f * worldSize) d -= worldSize;
    if (d < -0.5f * worldSize) d += worldSize;
    return d;
}

static inline Vec3 normalize(const Vec3& v, const float eps) {
    float l2 = sqrLen(v);
    if (l2 < eps)
        return {0.0f, 0.0f, 0.0f};
    float inv = 1.0f / std::sqrt(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}

static inline float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
