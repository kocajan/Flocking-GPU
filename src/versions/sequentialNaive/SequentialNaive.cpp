#include <cmath>
#include <algorithm>

#include "versions/sequentialNaive/SequentialNaive.hpp"
#include "versions/sequentialNaive/SequentialNaiveParameters.hpp" 


void resolveBasicBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx);

void resolvePredatorBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveObstacleAndWallAvoidance(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveDynamics(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveCollisions(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveMouseInteraction(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveRest(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveWallCollisions(SequentialNaiveParameters& params, int currentBoidIdx);

void resolveObstacleCollisions(SequentialNaiveParameters& params, int currentBoidIdx);


static inline float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static inline Vec3 normalize(const Vec3& v, const float EPS = 1e-5f) {
    float l2 = sqrLen(v);
    if (l2 < EPS)
        return {0.0f, 0.0f, 0.0f};
    float inv = 1.0f / std::sqrt(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}

void simulationStepSequentialNaive(SequentialNaiveParameters& params) {
    for (int currentBoidIdx = 0; currentBoidIdx < params.boidCount; ++currentBoidIdx) {
        // Get reference to current boid
        Boid& b = params.boids[currentBoidIdx];

        // Skip obstacles
        if (b.type == BoidType::Obstacle || b.type == BoidType::Empty)
            continue;

        // Zero acceleration
        b.acc = {0,0,0};

        if (b.type == BoidType::Basic) {
            resolveBasicBoidBehavior(
                params,
                currentBoidIdx
            );
        } else if (b.type == BoidType::Predator) {
            resolvePredatorBoidBehavior(
                params,
                currentBoidIdx
            );
        } else {
            // Unknown boid type
            printf("Warning: Unknown boid type encountered in simulation step. (type=%d)\n", static_cast<int>(b.type));
            continue;
        }
        resolveRest(
            params,
            currentBoidIdx
        );
    }
}

void resolveObstacleAndWallAvoidance(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid
    Boid& b = params.boids[currentBoidIdx];

    // Get parameters that depend on boid type
    const float rBoid = (b.type == BoidType::Basic) ? params.basicBoidRadius : params.predatorRadius;
    const float visualRange = (b.type == BoidType::Basic) ? params.visualRangeBasic : params.visualRangePredator;

    // Resolve obstacle avoidance
    Vec3 obstacleDirSum{0,0,0};
    float obstacleWeightSum = 0.0f;
    float obstacleCount = 0;
    for (size_t obsIdx : params.obstacleBoidIndices) {
        const Boid& obs = params.boids[obsIdx];

        Vec3 diff = {
            b.pos.x - obs.pos.x,
            b.pos.y - obs.pos.y,
            0.0f
        };

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
        b.acc.x += avoidDir.x * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        b.acc.y += avoidDir.y * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
        b.acc.z += avoidDir.z * params.maxForce * averageWeight * params.obstacleAvoidanceMultiplier;
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
    repelFromWall(b.pos.x - rBoid, 1.0f, b.acc.x);

    // Right wall
    repelFromWall((params.worldX - rBoid) - b.pos.x, -1.0f, b.acc.x);

    // Bottom wall
    repelFromWall(b.pos.y - rBoid, 1.0f, b.acc.y);

    // Top wall
    repelFromWall((params.worldY - rBoid) - b.pos.y, -1.0f, b.acc.y);

    if (!params.is2D)
    {
        // Floor
        repelFromWall(b.pos.z - rBoid, 1.0f, b.acc.z);

        // Ceiling
        repelFromWall((params.worldZ - rBoid) - b.pos.z, -1.0f, b.acc.z);
    }
}

void resolveBasicBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid
    Boid& b = params.boids[currentBoidIdx];

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
        const Boid& o = params.boids[otherIdx];

        // Compute distance vector
        Vec3 d = {
            o.pos.x - b.pos.x,
            o.pos.y - b.pos.y,
            o.pos.z - b.pos.z
        };

        // Get squared distance
        float d2 = sqrLen(d);

        // Skip if out of visual range
        if (d2 > params.visualRangeBasic2)
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
            personalSpace.x -= (o.pos.x - b.pos.x) * invd;
            personalSpace.y -= (o.pos.y - b.pos.y) * invd;
            personalSpace.z -= (o.pos.z - b.pos.z) * invd;
        } else if (dist >= params.basicBoidRadius * 2.0f) {
            // Cohesion - full vision range but outside protected range
            positionSum.x += o.pos.x;
            positionSum.y += o.pos.y;
            positionSum.z += o.pos.z;
            distantNeighborCount++;
        }

        // Alignment - full vision range
        velocitySum.x += o.vel.x;
        velocitySum.y += o.vel.y;
        velocitySum.z += o.vel.z;
    }

    Vec3 cohesionForce{0,0,0};
    Vec3 alignmentForce{0,0,0};
    if (distantNeighborCount > 0) {
        float invN = 1.0f / distantNeighborCount;

        // Cohesion - steer toward average position
        Vec3 avgPos = {
            positionSum.x * invN,
            positionSum.y * invN,
            positionSum.z * invN
        };
        cohesionForce = {
            avgPos.x - b.pos.x,
            avgPos.y - b.pos.y,
            avgPos.z - b.pos.z
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
            avgVel.x - b.vel.x,
            avgVel.y - b.vel.y,
            avgVel.z - b.vel.z
        };
    }

    // Move toward local target
    Vec3 toTarget = {
        b.targetPoint.x - b.pos.x,
        b.targetPoint.y - b.pos.y,
        b.targetPoint.z - b.pos.z
    };
    if (params.is2D)
        toTarget.z = 0.0f;
    float toTargetDist2 = sqrLen(toTarget);
    
    // Recalculate the targetWeight to include the squared distance to target
    if (toTargetDist2 < params.eps)
        toTargetDist2 = params.eps;
    float distanceFactor = toTargetDist2 / params.maxDistanceBetweenPoints;
    float adjustedTargetWeight = params.targetAttractionWeightBasic * distanceFactor;

    // Create for for cruising in the current velocity with the desired cruising speed
    Vec3 currentSpeedDir = normalize(b.vel, params.eps);
    Vec3 cruisingVel = {
        currentSpeedDir.x * params.cruisingSpeedBasic,
        currentSpeedDir.y * params.cruisingSpeedBasic,
        currentSpeedDir.z * params.cruisingSpeedBasic
    };
    Vec3 cruisingForce = {
        cruisingVel.x - b.vel.x,
        cruisingVel.y - b.vel.y,
        cruisingVel.z - b.vel.z
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
    Vec3 cruisingForceW = makeWeightedForce(cruisingForce, 0.5f);

    // Average the forces (just sum - they are already weighted)
    Vec3 averageForce = {
        cohesionForceW.x + alignmentForceW.x + separationForceW.x + targetForceW.x + cruisingForceW.x,
        cohesionForceW.y + alignmentForceW.y + separationForceW.y + targetForceW.y + cruisingForceW.y,
        cohesionForceW.z + alignmentForceW.z + separationForceW.z + targetForceW.z + cruisingForceW.z
    };
    float numForces = 5.0f;
    averageForce.x /= numForces;
    averageForce.y /= numForces;
    averageForce.z /= numForces;

    // Apply the average force to acceleration
    b.acc.x += averageForce.x;
    b.acc.y += averageForce.y;
    b.acc.z += averageForce.z;

    // Predator avoidance — simple random flee away from predators
    Vec3 predAvoidanceDir{0,0,0};
    int numPredators = 0;
    for (size_t predIdx : params.predatorBoidIndices) {
        // Get reference to predator boid
        Boid& pred = params.boids[predIdx];

        // Compute distance vector
        Vec3 distVect = {
            b.pos.x - pred.pos.x,
            b.pos.y - pred.pos.y,
            b.pos.z - pred.pos.z
        };

        float dist = std::sqrt(sqrLen(distVect));
        if (dist < params.eps)
            dist = params.eps;

        // Save info for predator chasing
        if (dist <= params.visualRangePredator) {
            if (pred.targetBoidIdx == -1 ||
                dist < pred.targetBoidDistance)
            {
                pred.targetBoidIdx = currentBoidIdx;
                pred.targetBoidDistance = dist;
            }
        }

        // Skip if out of visual range
        if (dist > params.visualRangeBasic)
            continue;

        // Increment predator count
        numPredators++;

        // Basic direction away from predator
        Vec3 away = normalize(distVect, params.eps);

        // Accumulate avoidance direction
        float invd = 1.0f / dist;
        predAvoidanceDir.x += away.x * invd;
        predAvoidanceDir.y += away.y * invd;
        predAvoidanceDir.z += away.z * invd;
    }

    // Apply escape impulse
    if (numPredators > 0) {
        // Panic - ignore flocking for this frame
        b.acc = {0,0,0};

        // Get escape direction
        Vec3 escape = normalize(predAvoidanceDir, params.eps);
        Vec3 escapeForceW = makeWeightedForce(escape, 1.0f);

        // Apply the escape force
        b.acc.x += escapeForceW.x;
        b.acc.y += escapeForceW.y;
        b.acc.z += escapeForceW.z;
    }
}

void resolvePredatorBoidBehavior(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid
    Boid& b = params.boids[currentBoidIdx];

    // First, try to maintain cruising speed
    Vec3 currentSpeedDir = normalize(b.vel, params.eps);
    Vec3 cruisingVel = {
        currentSpeedDir.x * params.cruisingSpeedPredator,
        currentSpeedDir.y * params.cruisingSpeedPredator,
        currentSpeedDir.z * params.cruisingSpeedPredator
    };
    Vec3 cruisingForce = {
        cruisingVel.x - b.vel.x,
        cruisingVel.y - b.vel.y,
        cruisingVel.z - b.vel.z
    };
    Vec3 cruisingForceW = {
        cruisingForce.x * (params.maxForce * 0.5f),
        cruisingForce.y * (params.maxForce * 0.5f),
        cruisingForce.z * (params.maxForce * 0.5f)
    };
    b.acc.x += cruisingForceW.x;
    b.acc.y += cruisingForceW.y;
    b.acc.z += cruisingForceW.z;

    // Chase the target boid if any
    if (b.targetBoidIdx != -1 && b.resting == false && b.stamina > 0.0f) {
        Boid& targetBoid = params.boids[b.targetBoidIdx];
        Vec3 toTarget = {
            targetBoid.pos.x - b.pos.x,
            targetBoid.pos.y - b.pos.y,
            targetBoid.pos.z - b.pos.z
        };
        if (params.is2D)
            toTarget.z = 0.0f;
        Vec3 toTargetN = normalize(toTarget, params.eps);
        float toTargetLen = std::sqrt(sqrLen(toTarget));
        b.acc.x = toTargetN.x * (toTargetLen * toTargetLen);
        b.acc.y = toTargetN.y * (toTargetLen * toTargetLen);
        b.acc.z = toTargetN.z * (toTargetLen * toTargetLen);
        b.stamina -= params.staminaDrainRatePredator * params.dt;
    } else if (b.stamina <= 0.0f && b.resting == false) {
        b.resting = true;
        b.stamina = 0.0f;
    } else if (b.resting == true && b.stamina > params.maxStaminaPredator) {
        b.resting = false;
        b.stamina = params.maxStaminaPredator;
    } else if (b.resting == true) {
        b.stamina += params.staminaRecoveryRatePredator * params.dt;
    }

    // Delete the target info for the next round
    b.targetBoidIdx = -1;
    b.targetBoidDistance = -1.0f;
    
}

void resolveDynamics(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid
    Boid& b = params.boids[currentBoidIdx];

    // Get parameters that depend on boid type
    const float maxSpeed = (b.type == BoidType::Basic) ? params.maxSpeedBasic : params.maxSpeedPredator;
    const float minSpeed = (b.type == BoidType::Basic) ? params.minSpeedBasic : params.minSpeedPredator;

    // Acceleration drag (simple linear drag)
    if (params.drag > 0.0f) {
        Vec3 stopAcc = {
            -b.vel.x / params.dt / params.numStepsToStopDueToMaxDrag,
            -b.vel.y / params.dt / params.numStepsToStopDueToMaxDrag,
            -b.vel.z / params.dt / params.numStepsToStopDueToMaxDrag
        };

        // apply fraction of full stop
        b.acc.x += stopAcc.x * params.drag;
        b.acc.y += stopAcc.y * params.drag;
        b.acc.z += stopAcc.z * params.drag;
    }

    // Current steering magnitude
    float accMagnitude = std::sqrt(sqrLen(b.acc));
    Vec3 accDir = normalize(b.acc, params.eps);

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
    b.acc.x = finalDir.x * accMagnitude;
    b.acc.y = finalDir.y * accMagnitude;
    b.acc.z = finalDir.z * accMagnitude;

    // Integrate
    b.vel.x += b.acc.x * params.dt;
    b.vel.y += b.acc.y * params.dt;
    b.vel.z += b.acc.z * params.dt;

    // Speed limits (Boids Lab style)
    float speed = std::sqrt(sqrLen(b.vel));
    if (speed > maxSpeed) {
        float s = maxSpeed / speed;
        b.vel.x *= s; 
        b.vel.y *= s; 
        b.vel.z *= s;
    } else if (speed < minSpeed) {
        float s = minSpeed / (speed + params.eps);
        b.vel.x *= s; 
        b.vel.y *= s; 
        b.vel.z *= s;
    }

    b.pos.x += b.vel.x * params.dt;
    b.pos.y += b.vel.y * params.dt;
    b.pos.z += b.vel.z * params.dt;
}

void resolveCollisions(SequentialNaiveParameters& params, int currentBoidIdx) {
    resolveWallCollisions(params, currentBoidIdx);
    resolveObstacleCollisions(params, currentBoidIdx);
}

void resolveWallCollisions(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid
    Boid& b = params.boids[currentBoidIdx];
    
    // Get parameters that depend on boid type
    const float rBoid = (b.type == BoidType::Basic) ? params.basicBoidRadius : params.predatorRadius;

    
    if (params.bounce) {
        if (b.pos.x < rBoid) { 
            b.pos.x = rBoid; 
            b.vel.x = -b.vel.x * params.bounceFactor;
            b.vel.y = b.vel.y * params.bounceFactor;
            b.vel.z = b.vel.z * params.bounceFactor;
        } else if (b.pos.x > params.worldX - rBoid) { 
            b.pos.x = params.worldX - rBoid; 
            b.vel.x = -b.vel.x * params.bounceFactor; 
            b.vel.y = b.vel.y * params.bounceFactor;
            b.vel.z = b.vel.z * params.bounceFactor;
        }

        if (b.pos.y < rBoid) { 
            b.pos.y = rBoid; 
            b.vel.y = -b.vel.y * params.bounceFactor; 
            b.vel.x = b.vel.x * params.bounceFactor;
            b.vel.z = b.vel.z * params.bounceFactor;
        } else if (b.pos.y > params.worldY - rBoid) { 
            b.pos.y = params.worldY - rBoid; 
            b.vel.y = -b.vel.y * params.bounceFactor; 
            b.vel.x = b.vel.x * params.bounceFactor;
            b.vel.z = b.vel.z * params.bounceFactor;
        }

        if (!params.is2D) {
            if (b.pos.z < rBoid) { 
                b.pos.z = rBoid; 
                b.vel.z = -b.vel.z * params.bounceFactor; 
                b.vel.x = b.vel.x * params.bounceFactor; 
                b.vel.y = b.vel.y * params.bounceFactor; 
            } else if (b.pos.z > params.worldZ - rBoid) { 
                b.pos.z = params.worldZ - rBoid; 
                b.vel.z = -b.vel.z * params.bounceFactor; 
                b.vel.x = b.vel.x * params.bounceFactor; 
                b.vel.y = b.vel.y * params.bounceFactor; 
            }
        }
    } else {
        if (b.pos.x < 0) {
            b.pos.x += params.worldX;
        } else if (b.pos.x >= params.worldX) {
            b.pos.x -= params.worldX;
        }
        
        if (b.pos.y < 0) {
            b.pos.y += params.worldY; 
        } else if (b.pos.y >= params.worldY) {
            b.pos.y -= params.worldY;
        }
        
        if (!params.is2D) {
            if (b.pos.z < 0) {
                b.pos.z += params.worldZ;
            } else if (b.pos.z >= params.worldZ) {
                b.pos.z -= params.worldZ;
            }
        }
    }
}

void resolveObstacleCollisions(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid
    Boid& b = params.boids[currentBoidIdx];

    // Get parameters that depend on boid type
    const float rBoid = (b.type == BoidType::Basic) ? params.basicBoidRadius : params.predatorRadius;

    // Check collisions with obstacles
    for (size_t obsIdx : params.obstacleBoidIndices) {
        const Boid& obs = params.boids[obsIdx];

        Vec3 diff = {
            b.pos.x - obs.pos.x,
            b.pos.y - obs.pos.y,
            0.0f
        };

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + params.obstacleRadius;
        float dist = std::sqrt(dist2) - combinedRadius;

        if (dist < 0.0f) {
            Vec3 n = normalize(diff);

            // Push boid out of obstacle
            b.pos.x += n.x * (-dist + params.eps);
            b.pos.y += n.y * (-dist + params.eps);
            b.pos.z += n.z * (-dist + params.eps);

            // Reflection calculation
            float vDotN = b.vel.x*n.x + b.vel.y*n.y + b.vel.z*n.z;

            // Reflected velocity (full bounce)
            Vec3 vReflect = {
                b.vel.x - 2.0f * vDotN * n.x,
                b.vel.y - 2.0f * vDotN * n.y,
                b.vel.z - 2.0f * vDotN * n.z
            };

            // Apply bounce factor
            b.vel.x = vReflect.x * params.bounceFactor;
            b.vel.y = vReflect.y * params.bounceFactor;
            b.vel.z = vReflect.z * params.bounceFactor;
        }
    }
}

void resolveMouseInteraction(SequentialNaiveParameters& params, int currentBoidIdx) {
    // Get reference to current boid and interaction
    Boid& b = params.boids[currentBoidIdx];
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
        b.acc.x = 0.0f;
        b.acc.y = 0.0f;
        b.acc.z = 0.0f;

        Vec3 diff = {
            b.pos.x - inter.point.x,
            b.pos.y - inter.point.y,
            b.pos.z - inter.point.z
        };

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
            b.acc.x -= weightedForce.x;
            b.acc.y -= weightedForce.y;
            b.acc.z -= weightedForce.z;
        } else {
            b.acc.x += weightedForce.x;
            b.acc.y += weightedForce.y;
            b.acc.z += weightedForce.z;
        }
    }
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
        params.boids[currentBoidIdx].pos.z = 0.0f;
        params.boids[currentBoidIdx].vel.z = 0.0f;
    }
}
