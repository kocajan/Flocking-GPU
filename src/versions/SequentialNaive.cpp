#include "versions/SequentialNaive.hpp"

#include <cmath>
#include <algorithm>


void resolveBasicBoidBehavior(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolvePredatorBoidBehavior(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveObstacleAndWallAvoidance(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveDynamics(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveCollisions(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveMouseInteraction(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveRest(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveWallCollisions(SimState& simState, const Config& simConfig, int currentBoidIdx);

void resolveObstacleCollisions(SimState& simState, const Config& simConfig, int currentBoidIdx);


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

void sequentialNaiveSimulationStep(SimState& simState, const Config& simConfig) {
    for (int currentBoidIdx = 0; currentBoidIdx < simState.boids.size(); ++currentBoidIdx) {
        // Get reference to current boid
        Boid& b = simState.boids[currentBoidIdx];

        // Skip obstacles
        if (b.type == BoidType::Obstacle || b.type == BoidType::Empty)
            continue;

        // Zero acceleration
        b.acc = {0,0,0};

        if (b.type == BoidType::Basic) {
            resolveBasicBoidBehavior(
                simState,
                simConfig,
                currentBoidIdx
            );
        } else if (b.type == BoidType::Predator) {
            resolvePredatorBoidBehavior(
                simState,
                simConfig,
                currentBoidIdx
            );
        } else {
            // Unknown boid type
            printf("Warning: Unknown boid type encountered in simulation step. (type=%d)\n", static_cast<int>(b.type));
            continue;
        }
        resolveRest(
            simState,
            simConfig,
            currentBoidIdx
        );
    }
}

void resolveObstacleAndWallAvoidance(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    Boid& b = simState.boids[currentBoidIdx];
    const bool  is2D = (simState.dimensions.string() == "2D");
    const float rBoid = (b.type == BoidType::Basic) ? simState.basicBoidRadius.number() : simState.predatorRadius.number();
    const float rObstacle = simState.obstacleRadius.number();
    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();
    const float eps = simState.eps.number();
    const std::vector<size_t>& obstacleBoidIndices = simState.obstacleBoidIndices;
    const float maxVisualRange = std::sqrt(worldX*worldX + worldY*worldY + (is2D ? 0.0f : worldZ*worldZ)); 

    // Unpack parameters from simConfig
    const float visualRangePercentage = (b.type == BoidType::Basic) ? simConfig.number("visionBasic") : simConfig.number("visionPredator");
    const bool bounce = simConfig.binary("bounce");
    const float visualRange = (visualRangePercentage / 100.0f) * maxVisualRange;
    const float maxForce = simConfig.number("maxForce");

    Vec3 obstacleDirSum{0,0,0};
    float obstacleWeightSum = 0.0f;
    float obstacleCount = 0;
    for (size_t obsIdx : obstacleBoidIndices) {
        const Boid& obs = simState.boids[obsIdx];

        Vec3 diff = {
            b.pos.x - obs.pos.x,
            b.pos.y - obs.pos.y,
            0.0f
        };

        float centerDist = std::sqrt(sqrLen(diff));
        float combinedRadius = rBoid + rObstacle;
        float surfaceDist = centerDist - combinedRadius;

        if (surfaceDist > visualRange)
            continue;
        obstacleCount++;

        Vec3 dir = normalize(diff, eps);

        // Proximity weight
        float weight = std::exp(-0.1 * surfaceDist);

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

        Vec3 avoidDir = normalize(avgDir, eps);

        // Apply as a single steering vector
        b.acc.x += avoidDir.x * maxForce * averageWeight * 500.0f;
        b.acc.y += avoidDir.y * maxForce * averageWeight * 500.0f;
        b.acc.z += avoidDir.z * maxForce * averageWeight * 500.0f;
    }

    // If the boid is close to walls, apply repelling force only if bounce is enabled
    if (!bounce) {
        return;
    }

    auto repelFromWall = [&](float d, float axisSign, float& accAxis)
    {
        if (d < visualRange) {
            float weight = std::exp(-d);
            accAxis += axisSign * (maxForce * weight * 1000.0f);
        }
    };

    // Left wall
    repelFromWall(b.pos.x - rBoid, 1.0f, b.acc.x);

    // Right wall
    repelFromWall((worldX - rBoid) - b.pos.x, -1.0f, b.acc.x);

    // Bottom wall
    repelFromWall(b.pos.y - rBoid, 1.0f, b.acc.y);

    // Top wall
    repelFromWall((worldY - rBoid) - b.pos.y, -1.0f, b.acc.y);

    if (!is2D)
    {
        // Floor
        repelFromWall(b.pos.z - rBoid, 1.0f, b.acc.z);

        // Ceiling
        repelFromWall((worldZ - rBoid) - b.pos.z, -1.0f, b.acc.z);
    }
}

void resolveBasicBoidBehavior(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    const bool  is2D = (simState.dimensions.string() == "2D");
    const float rBoid = simState.basicBoidRadius.number();
    const float eps = simState.eps.number();
    const uint64_t basicBoidCount = simState.basicBoidCount;
    Boid& b = simState.boids[currentBoidIdx];

    // Unpack parameters from simConfig
    const float visualRangeBasic = simConfig.number("visionBasic");
    const float visualRangePredator = simConfig.number("visionPredator");
    const float cohesionWeight = std::clamp(simConfig.number("cohesionBasic") / 100.0f, 0.0f, 1.0f);
    const float alignmentWeight = std::clamp(simConfig.number("alignmentBasic") / 100.0f, 0.0f, 1.0f);
    const float separationWeight = std::clamp(simConfig.number("separationBasic") / 100.0f, 0.0f, 1.0f);
    const float targetWeight = std::clamp(simConfig.number("targetAttractionBasic") / 100.0f, 0.0f, 1.0f);
    const float neighborAccuracy = simConfig.number("accuracy");
    const float maxForce = simConfig.number("maxForce");
    const bool bounce = simConfig.binary("bounce");

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        return Vec3{
            dir.x * (maxForce * weight),
            dir.y * (maxForce * weight),
            dir.z * (maxForce * weight)
        };
    };

    // Calculate additional parameters
    const float rBoid2 = rBoid * rBoid;
    const float visualRangeBasic2 = visualRangeBasic * visualRangeBasic;
    const uint64_t maxNeighbors = static_cast<uint64_t>(basicBoidCount * (neighborAccuracy / 100.0f));
    float maxDistanceBetweenPoints = 0.0f;
    if (is2D) {
        maxDistanceBetweenPoints = std::sqrt(simState.worldX.number()*simState.worldX.number() + simState.worldY.number()*simState.worldY.number());
    } else {
        maxDistanceBetweenPoints = std::sqrt(simState.worldX.number()*simState.worldX.number() + simState.worldY.number()*simState.worldY.number() + simState.worldZ.number()*simState.worldZ.number());
    }
    if (!bounce) {
        maxDistanceBetweenPoints *= 0.5f;
    }
    float maxDistanceBetweenPoints2 = maxDistanceBetweenPoints * maxDistanceBetweenPoints;
    
    // Initialize accumulators
    Vec3 personalSpace{0,0,0};
    Vec3 positionSum{0,0,0};
    Vec3 velocitySum{0,0,0};
    int neighborCount = 0;
    int distantNeighborCount = 0;

    // Analyze other boids
    for (size_t otherIdx : simState.basicBoidIndices) {
        // Break if reached max neighbors
        if (neighborCount >= maxNeighbors)
            break;
            
        // Skip self
        if (otherIdx == currentBoidIdx) continue;

        // Get reference to other boid
        const Boid& o = simState.boids[otherIdx];

        // Compute distance vector
        Vec3 d = {
            o.pos.x - b.pos.x,
            o.pos.y - b.pos.y,
            o.pos.z - b.pos.z
        };

        // Get squared distance
        float d2 = sqrLen(d);

        // Skip if out of visual range
        if (d2 > visualRangeBasic2)
            continue;

        // Compute distance and avoid zero-length
        float dist = std::sqrt(d2);
        if (dist < eps)
            dist = eps;

        // Increment neighbor count
        neighborCount++;

        // Separation - only inside protected range
        if (dist < rBoid * 2.0f) {
            float invd = 1.0f / dist;
            personalSpace.x -= (o.pos.x - b.pos.x) * invd;
            personalSpace.y -= (o.pos.y - b.pos.y) * invd;
            personalSpace.z -= (o.pos.z - b.pos.z) * invd;
        } else if (dist >= rBoid * 2.0f) {
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
    if (is2D)
        toTarget.z = 0.0f;
    float toTargetDist2 = sqrLen(toTarget);
    
    // Recalculate the targetWeight to include the squared distance to target
    if (toTargetDist2 < eps)
        toTargetDist2 = eps;
    float distanceFactor = toTargetDist2 / maxDistanceBetweenPoints;
    float adjustedTargetWeight = targetWeight * distanceFactor;
    float adjustedTargetWeightClamped = std::clamp(adjustedTargetWeight, 0.0f, 1.0f);

    // Get directions from the forces
    Vec3 cohesionDir = normalize(cohesionForce, eps);
    Vec3 alignmentDir = normalize(alignmentForce, eps);
    Vec3 separationDir = normalize(personalSpace, eps);
    Vec3 targetDir = normalize(toTarget, eps);

    // Get weighted forces
    Vec3 cohesionForceW = makeWeightedForce(cohesionDir, cohesionWeight);
    Vec3 alignmentForceW = makeWeightedForce(alignmentDir, alignmentWeight);
    Vec3 separationForceW = makeWeightedForce(separationDir, separationWeight);
    Vec3 targetForceW = makeWeightedForce(targetDir, adjustedTargetWeight);

    // Average the forces (just sum - they are already weighted)
    Vec3 averageForce = {
        cohesionForceW.x + alignmentForceW.x + separationForceW.x + targetForceW.x,
        cohesionForceW.y + alignmentForceW.y + separationForceW.y + targetForceW.y,
        cohesionForceW.z + alignmentForceW.z + separationForceW.z + targetForceW.z
    };
    float numForces = 4.0f;
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
    for (size_t predIdx : simState.predatorBoidIndices) {
        // Get reference to predator boid
        Boid& pred = simState.boids[predIdx];

        // Compute distance vector
        Vec3 distVect = {
            b.pos.x - pred.pos.x,
            b.pos.y - pred.pos.y,
            b.pos.z - pred.pos.z
        };

        float dist = std::sqrt(sqrLen(distVect));
        if (dist < eps)
            dist = eps;

        // Save info for predator chasing
        if (dist <= visualRangePredator) {
            if (pred.targetBoidIdx == -1 ||
                dist < pred.targetBoidDistance)
            {
                pred.targetBoidIdx = currentBoidIdx;
                pred.targetBoidDistance = dist;
            }
        }

        // Skip if out of visual range
        if (dist > visualRangeBasic)
            continue;

        // Increment predator count
        numPredators++;

        // Basic direction away from predator
        Vec3 away = normalize(distVect, eps);

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
        Vec3 escape = normalize(predAvoidanceDir, eps);
        Vec3 escapeForceW = makeWeightedForce(escape, 1.0f);

        // Apply the escape force
        b.acc.x += escapeForceW.x;
        b.acc.y += escapeForceW.y;
        b.acc.z += escapeForceW.z;
    }
}

void resolvePredatorBoidBehavior(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    const bool  is2D = (simState.dimensions.string() == "2D");
    Boid& b = simState.boids[currentBoidIdx];
    const float eps = simState.eps.number();
    const float dt = simState.dt.number();

    // Unpack parameters from simConfig
    const float maxSpeed = simConfig.number("maxSpeedPredator");
    const float minSpeed = simConfig.number("minSpeedPredator");
    const float maxForce = simConfig.number("maxForce");
    const float maxStamina = 100.0f;
    const float staminaRecoveryRate = 10.0f;
    const float staminaDrainRate = 20.0f;

    // First make random wandering at minimum speed - create the vector randomly so goes the minimum speed
    if (std::sqrt(sqrLen(b.vel)) < minSpeed) {
        b.acc.x += ((float)rand() / RAND_MAX - 0.5f) * 10.0f;
        b.acc.y += ((float)rand() / RAND_MAX - 0.5f) * 10.0f;
        b.acc.z += ((float)rand() / RAND_MAX - 0.5f) * 10.0f;
    }

    // Chase the target boid if any
    if (b.targetBoidIdx != -1 && b.resting == false && b.stamina > 0.0f) {
        Boid& targetBoid = simState.boids[b.targetBoidIdx];
        Vec3 toTarget = {
            targetBoid.pos.x - b.pos.x,
            targetBoid.pos.y - b.pos.y,
            targetBoid.pos.z - b.pos.z
        };
        if (is2D)
            toTarget.z = 0.0f;
        Vec3 toTargetN = normalize(toTarget, eps);
        float toTargetLen = std::sqrt(sqrLen(toTarget));
        b.acc.x = toTargetN.x * (toTargetLen * toTargetLen);
        b.acc.y = toTargetN.y * (toTargetLen * toTargetLen);
        b.acc.z = toTargetN.z * (toTargetLen * toTargetLen);
        b.stamina -= staminaDrainRate * dt;
    } else if (b.stamina <= 0.0f && b.resting == false) {
        b.resting = true;
        b.stamina = 0.0f;
    } else if (b.resting == true && b.stamina > maxStamina) {
        b.resting = false;
        b.stamina = maxStamina;
    } else if (b.resting == true) {
        b.stamina += staminaRecoveryRate * dt;
    }

    // Delete the target info for the next round
    b.targetBoidIdx = -1;
    b.targetBoidDistance = -1.0f;
    
}

void resolveDynamics(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    Boid& b = simState.boids[currentBoidIdx];
    const float eps = simState.eps.number();
    const float dt = simState.dt.number();

    // Unpack parameters from simConfig
    const float maxSpeed = (b.type == BoidType::Basic) ? simConfig.number("maxSpeedBasic") : simConfig.number("maxSpeedPredator");
    const float minSpeed = (b.type == BoidType::Basic) ? simConfig.number("minSpeedBasic") : simConfig.number("minSpeedPredator");
    const float drag = simConfig.number("drag");
    const float noise = simConfig.number("noise");
    const float maxForce = simConfig.number("maxForce");

    // Drag + noise
    float dragPct = std::clamp(drag / 100.0f, 0.0f, 1.0f);

    // Acceleration drag (simple linear drag)
    float numStepsToStop = 100.0f;
    if (dragPct > 0.0f) {
        Vec3 stopAcc = {
            -b.vel.x / dt / numStepsToStop,
            -b.vel.y / dt / numStepsToStop,
            -b.vel.z / dt / numStepsToStop
        };

        // apply fraction of full stop
        b.acc.x += stopAcc.x * dragPct;
        b.acc.y += stopAcc.y * dragPct;
        b.acc.z += stopAcc.z * dragPct;
    }

    float noisePct = std::clamp(noise / 100.0f, 0.0f, 1.0f);

    // Current steering magnitude
    float accMagnitude = std::sqrt(sqrLen(b.acc));
    Vec3 accDir = normalize(b.acc, eps);

    // Random unit vector
    Vec3 randVec = {
        ((float)rand()/RAND_MAX - 0.5f),
        ((float)rand()/RAND_MAX - 0.5f),
        ((float)rand()/RAND_MAX - 0.5f)
    };
    Vec3 randDir = normalize(randVec, eps);

    // blend steering vs randomness
    Vec3 blended = {
        accDir.x * (1.0f - noisePct) + randDir.x * noisePct,
        accDir.y * (1.0f - noisePct) + randDir.y * noisePct,
        accDir.z * (1.0f - noisePct) + randDir.z * noisePct
    };

    Vec3 finalDir = normalize(blended, eps);

    // preserve magnitude
    b.acc.x = finalDir.x * accMagnitude;
    b.acc.y = finalDir.y * accMagnitude;
    b.acc.z = finalDir.z * accMagnitude;

    // Integrate
    b.vel.x += b.acc.x * dt;
    b.vel.y += b.acc.y * dt;
    b.vel.z += b.acc.z * dt;

    // Speed limits (Boids Lab style)
    float speed = std::sqrt(sqrLen(b.vel));
    if (speed > maxSpeed) {
        float s = maxSpeed / speed;
        b.vel.x *= s; 
        b.vel.y *= s; 
        b.vel.z *= s;
    } else if (speed < minSpeed) {
        float s = minSpeed / (speed + eps);
        b.vel.x *= s; 
        b.vel.y *= s; 
        b.vel.z *= s;
    }

    b.pos.x += b.vel.x * dt;
    b.pos.y += b.vel.y * dt;
    b.pos.z += b.vel.z * dt;
}

void resolveCollisions(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    resolveWallCollisions(simState, simConfig, currentBoidIdx);
    resolveObstacleCollisions(simState, simConfig, currentBoidIdx);
}

void resolveWallCollisions(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    Boid& b = simState.boids[currentBoidIdx];
    const bool  is2D = (simState.dimensions.string() == "2D");
    const float rBoid = (b.type == BoidType::Basic) ? simState.basicBoidRadius.number() : simState.predatorRadius.number();
    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    // Unpack parameters from simConfig
    const bool bounce = simConfig.binary("bounce");
    float bounceFactor = simConfig.number("bounceFactor") / 100.0f;
    
    if (bounce) {
        if (b.pos.x < rBoid) { 
            b.pos.x = rBoid; 
            b.vel.x = -b.vel.x * bounceFactor;
            b.vel.y = b.vel.y * bounceFactor;
            b.vel.z = b.vel.z * bounceFactor;
        } else if (b.pos.x > worldX - rBoid) { 
            b.pos.x = worldX - rBoid; 
            b.vel.x = -b.vel.x * bounceFactor; 
            b.vel.y = b.vel.y * bounceFactor;
            b.vel.z = b.vel.z * bounceFactor;
        }

        if (b.pos.y < rBoid) { 
            b.pos.y = rBoid; 
            b.vel.y = -b.vel.y * bounceFactor; 
            b.vel.x = b.vel.x * bounceFactor;
            b.vel.z = b.vel.z * bounceFactor;
        } else if (b.pos.y > worldY - rBoid) { 
            b.pos.y = worldY - rBoid; 
            b.vel.y = -b.vel.y * bounceFactor; 
            b.vel.x = b.vel.x * bounceFactor;
            b.vel.z = b.vel.z * bounceFactor;
        }

        if (!is2D) {
            if (b.pos.z < rBoid) { 
                b.pos.z = rBoid; 
                b.vel.z = -b.vel.z * bounceFactor; 
                b.vel.x = b.vel.x * bounceFactor; 
                b.vel.y = b.vel.y * bounceFactor; 
            } else if (b.pos.z > worldZ - rBoid) { 
                b.pos.z = worldZ - rBoid; 
                b.vel.z = -b.vel.z * bounceFactor; 
                b.vel.x = b.vel.x * bounceFactor; 
                b.vel.y = b.vel.y * bounceFactor; 
            }
        }
    } else {
        if (b.pos.x < 0) {
            b.pos.x += worldX;
        } else if (b.pos.x >= worldX) {
            b.pos.x -= worldX;
        }
        
        if (b.pos.y < 0) {
            b.pos.y += worldY; 
        } else if (b.pos.y >= worldY) {
            b.pos.y -= worldY;
        }
        
        if (!is2D) {
            if (b.pos.z < 0) {
                b.pos.z += worldZ;
            } else if (b.pos.z >= worldZ) {
                b.pos.z -= worldZ;
            }
        }
    }
}

void resolveObstacleCollisions(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    Boid& b = simState.boids[currentBoidIdx];
    const float rBoid = simState.basicBoidRadius.number();
    const float rObstacle = simState.obstacleRadius.number();
    const float eps = simState.eps.number();
    const std::vector<size_t>& obstacleBoidIndices = simState.obstacleBoidIndices;

    // Unpack parameters from simConfig
    const float bounceFactor = simConfig.number("bounceFactor") / 100.0f;

    // Check collisions with obstacles
    for (size_t obsIdx : obstacleBoidIndices) {
        const Boid& obs = simState.boids[obsIdx];

        Vec3 diff = {
            b.pos.x - obs.pos.x,
            b.pos.y - obs.pos.y,
            0.0f
        };

        float dist2 = sqrLen(diff);
        float combinedRadius = rBoid + rObstacle;
        float dist = std::sqrt(dist2) - combinedRadius;

        if (dist < 0.0f) {
            Vec3 n = normalize(diff);

            // Push boid out of obstacle
            b.pos.x += n.x * (-dist + eps);
            b.pos.y += n.y * (-dist + eps);
            b.pos.z += n.z * (-dist + eps);

            // Reflection calculation
            float vDotN = b.vel.x*n.x + b.vel.y*n.y + b.vel.z*n.z;

            // Reflected velocity (full bounce)
            Vec3 vReflect = {
                b.vel.x - 2.0f * vDotN * n.x,
                b.vel.y - 2.0f * vDotN * n.y,
                b.vel.z - 2.0f * vDotN * n.z
            };

            // Apply bounce factor
            float e = std::clamp(bounceFactor, 0.0f, 1.0f);
            b.vel.x = vReflect.x * e;
            b.vel.y = vReflect.y * e;
            b.vel.z = vReflect.z * e;
        }
    }
}

void resolveMouseInteraction(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Unpack parameters from simState
    Boid& b = simState.boids[currentBoidIdx];
    const Interaction& inter = simState.interaction;
    const float eps = simState.eps.number();
    const std::string dimensions = simState.dimensions.string();
    const bool is2D = (dimensions == "2D");

    // Unpack parameters from simConfig
    const float maxForce = simConfig.number("maxForce");
    const bool bounce = simConfig.binary("bounce");

    // Define helper to make weighted forces
    auto makeWeightedForce = [&](const Vec3& dir, float weight) {
        return Vec3{
            dir.x * (maxForce * weight) * 1000.0f,
            dir.y * (maxForce * weight) * 1000.0f,
            dir.z * (maxForce * weight) * 1000.0f
        };
    };

    float maxDistanceBetweenPoints = 0.0f;
    if (is2D) {
        maxDistanceBetweenPoints = std::sqrt(simState.worldX.number()*simState.worldX.number() + simState.worldY.number()*simState.worldY.number());
    } else {
        maxDistanceBetweenPoints = std::sqrt(simState.worldX.number()*simState.worldX.number() + simState.worldY.number()*simState.worldY.number() + simState.worldZ.number()*simState.worldZ.number());
    }
    if (!bounce) {
        maxDistanceBetweenPoints *= 0.5f;
    }
    float maxDistanceBetweenPoints2 = maxDistanceBetweenPoints * maxDistanceBetweenPoints;

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
        if (dist2 < eps)
            dist2 = eps;

        // Create weight based on distance
        float weight = dist2 / maxDistanceBetweenPoints2;
        if (weight < 0.0f)
            weight = 0.0f;

        // Get normalized direction
        Vec3 dir = normalize(diff, eps);

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

void resolveRest(SimState& simState, const Config& simConfig, int currentBoidIdx) {
    // Resolve mouse interactions
    resolveMouseInteraction(simState, simConfig, currentBoidIdx);

    // Resolve obstacle avoidance
    resolveObstacleAndWallAvoidance(simState, simConfig, currentBoidIdx);

    // Resolve dynamics
    resolveDynamics(simState, simConfig, currentBoidIdx);

    // Resolve wall interactions
    resolveCollisions(simState, simConfig, currentBoidIdx);

    if (simState.dimensions.string() == "2D") {
        simState.boids[currentBoidIdx].pos.z = 0.0f;
        simState.boids[currentBoidIdx].vel.z = 0.0f;
    }
}
