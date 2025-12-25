#include "versions/Sequential.hpp"

#include <cmath>

static constexpr float EPS = 1e-5f;

static inline float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static inline Vec3 normalize(const Vec3& v) {
    float l2 = sqrLen(v);
    if (l2 < EPS)
        return {0.0f, 0.0f, 0.0f};

    float inv = 1.0f / std::sqrt(l2);
    return { v.x * inv, v.y * inv, v.z * inv };
}

void sequentialSimulationStep(SimState& simState, const Config& simConfig) {
    const float dt   = simState.dt.number();
    const bool  is2D = (simState.dimensions.string() == "2D");
    const bool bounce = simConfig.binary("bounce");

    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    // Load other parameters as needed
    const float visionRadius = simConfig.number("vision");
    const float alignmentWeight = simConfig.number("alignment");
    const float cohesionWeight = simConfig.number("cohesion");
    const float separationWeight = simConfig.number("separation");
    const float maxForce = simConfig.number("max_force");
    const float maxSpeed = simConfig.number("max_speed");
    const float minSpeed = simConfig.number("min_speed");
    const float drag = simConfig.number("drag");
    const float noise = simConfig.number("noise");

    // Get boid indices
    std::vector<size_t> obstacleBoidIndices = simState.obstacleBoidIndices;
    std::vector<size_t> predatorBoidIndices = simState.predatorBoidIndices;

    // radii
    const float rObstacle = simState.obstacleRadius.number();
    const float rBasic = simState.basicBoidRadius.number();
    const float rPred = simState.predatorRadius.number();

    // walls
    constexpr float WALL_BOUNCE    = 0.6f;

    // obstacle bounce
    constexpr float BOUNCE_FACTOR = 0.6f;

    for (int currentBoidIdx = 0; currentBoidIdx < simState.boids.size(); ++currentBoidIdx) {
        Boid& b = simState.boids[currentBoidIdx];
        b.acc = {0,0,0};

        if (b.type == BoidType::Obstacle)
            continue;

        float rBoid = (b.type == BoidType::Predator) ? rPred : rBasic;

        // ##############################################################
        // Accumulators
        Vec3 alignmentSum {0.0f, 0.0f, 0.0f};
        Vec3 cohesionSum  {0.0f, 0.0f, 0.0f};
        Vec3 separationSum{0.0f, 0.0f, 0.0f};
        int neighborCount = 0;

        float visionRadius2 = visionRadius * visionRadius;

        for (size_t otherIdx : simState.basicBoidIndices) {
            if (otherIdx == currentBoidIdx) continue;

            const Boid& other = simState.boids[otherIdx];

            Vec3 toOther = {
                other.pos.x - b.pos.x,
                other.pos.y - b.pos.y,
                other.pos.z - b.pos.z
            };

            float dist2 = sqrLen(toOther);
            if (dist2 > visionRadius2) continue;

            float dist = std::sqrt(dist2);
            if (dist <= 0.0001f) continue;

            neighborCount++;

            Vec3 n = {
                toOther.x / dist,
                toOther.y / dist,
                toOther.z / dist
            };

            // Alignment sum
            alignmentSum.x += other.vel.x;
            alignmentSum.y += other.vel.y;
            alignmentSum.z += other.vel.z;

            // Cohesion sum
            cohesionSum.x += other.pos.x;
            cohesionSum.y += other.pos.y;
            cohesionSum.z += other.pos.z;

            // Separation — inverse square weighting
            float sepStrength = 1.0f / dist2;
            separationSum.x -= n.x * sepStrength;
            separationSum.y -= n.y * sepStrength;
            separationSum.z -= n.z * sepStrength;
        }

        if (neighborCount > 0) {
            float invN = 1.0f / neighborCount;

            // ----- Alignment (heading-based, stable) -----
            Vec3 avgVel = {
                alignmentSum.x * invN,
                alignmentSum.y * invN,
                alignmentSum.z * invN
            };

            Vec3 desiredDir = normalize(avgVel);
            Vec3 desired = {
                desiredDir.x * maxSpeed,
                desiredDir.y * maxSpeed,
                desiredDir.z * maxSpeed
            };

            Vec3 alignSteer = {
                (desired.x - b.vel.x) * alignmentWeight,
                (desired.y - b.vel.y) * alignmentWeight,
                (desired.z - b.vel.z) * alignmentWeight
            };

            // ----- Cohesion (unchanged) -----
            Vec3 center = {
                cohesionSum.x * invN,
                cohesionSum.y * invN,
                cohesionSum.z * invN
            };

            Vec3 toCenter = {
                center.x - b.pos.x,
                center.y - b.pos.y,
                center.z - b.pos.z
            };

            float toCenterLen2 = sqrLen(toCenter);
            Vec3 cohesionSteer {0.0f, 0.0f, 0.0f};
            if (toCenterLen2 > 0.0001f) {
                float toCenterLen = std::sqrt(toCenterLen2);
                Vec3 dirToCenter = {
                    toCenter.x / toCenterLen,
                    toCenter.y / toCenterLen,
                    toCenter.z / toCenterLen
                };
                cohesionSteer.x = dirToCenter.x * cohesionWeight;
                cohesionSteer.y = dirToCenter.y * cohesionWeight;
                cohesionSteer.z = dirToCenter.z * cohesionWeight;
            }

            // ----- Separation (not averaged, strong repulsion) -----
            Vec3 separationSteer {0.0f, 0.0f, 0.0f};
            float sepLen2 = sqrLen(separationSum);
            if (sepLen2 > EPS) {
                float sepLen = std::sqrt(sepLen2);
                Vec3 dir = {
                    separationSum.x / sepLen,
                    separationSum.y / sepLen,
                    separationSum.z / sepLen
                };
                separationSteer.x = dir.x * separationWeight;
                separationSteer.y = dir.y * separationWeight;
                separationSteer.z = dir.z * separationWeight;
            }

            // ----- Combine & clamp -----
            Vec3 steer = {
                alignSteer.x + cohesionSteer.x + separationSteer.x,
                alignSteer.y + cohesionSteer.y + separationSteer.y,
                alignSteer.z + cohesionSteer.z + separationSteer.z
            };

            float steerLen2 = sqrLen(steer);
            if (steerLen2 > maxForce * maxForce) {
                float inv = maxForce / std::sqrt(steerLen2);
                steer.x *= inv;
                steer.y *= inv;
                steer.z *= inv;
            }

            b.acc.x += steer.x;
            b.acc.y += steer.y;
            b.acc.z += steer.z;
        }

        // ##############################################################

        // Apply interaction from mouse
        const Interaction& inter = simState.interaction;
        const InteractionType interType = inter.type;
        if (interType != InteractionType::Empty) {
            float dx = b.pos.x - inter.point.x;
            float dy = b.pos.y - inter.point.y;
            float dz = b.pos.z - inter.point.z;

            float dist2 = dx*dx + dy*dy + dz*dz;
            if (dist2 < 1e-6f) dist2 = 1e-6f;

            float invDist = 1.0f / std::sqrt(dist2);

            float nx = dx * invDist;
            float ny = dy * invDist;
            float nz = dz * invDist;

            float forceMag = 10000000.0f / dist2;
            if (forceMag > 100.0f)
                forceMag = 100.0f;

            if (interType == InteractionType::Attract) {
                b.acc.x -= nx * forceMag;
                b.acc.y -= ny * forceMag;
                b.acc.z -= nz * forceMag;
            }
            else if (interType == InteractionType::Repel) {
                b.acc.x += nx * forceMag;
                b.acc.y += ny * forceMag;
                b.acc.z += nz * forceMag;
            }
        }

        // For each boid, calculate acceleration due to obstacles
        for (size_t obsIdx : obstacleBoidIndices) {
            const Boid& obs = simState.boids[obsIdx];

            Vec3 toBoid = {
                b.pos.x - obs.pos.x,
                b.pos.y - obs.pos.y,
                b.pos.z - obs.pos.z
            };

            float dist2 = sqrLen(toBoid);
            float combinedRadius = rBoid + rObstacle;
            float dist = std::sqrt(dist2) - combinedRadius;

            if (dist < 0.0f) {
                Vec3 n = normalize(toBoid);
                b.pos.x += n.x * (-dist + EPS);
                b.pos.y += n.y * (-dist + EPS);
                b.pos.z += n.z * (-dist + EPS);
                float vDotN = b.vel.x * n.x + b.vel.y * n.y + b.vel.z * n.z;
                b.vel.x -= (1.0f + BOUNCE_FACTOR) * vDotN * n.x;
                b.vel.y -= (1.0f + BOUNCE_FACTOR) * vDotN * n.y;
                b.vel.z -= (1.0f + BOUNCE_FACTOR) * vDotN * n.z;
            } else {
                float forceMag = 1000.0f * std::exp(-dist / 5.0f);
                Vec3 n = normalize(toBoid);
                b.acc.x += n.x * forceMag;
                b.acc.y += n.y * forceMag;
                b.acc.z += n.z * forceMag;
            }
        }

        // Use drag and noise
        b.acc.x += -b.vel.x * drag;
        b.acc.y += -b.vel.y * drag;
        b.acc.z += -b.vel.z * drag;

        b.acc.x += ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * noise;
        b.acc.y += ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * noise;
        b.acc.z += ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * noise;

        // Integrate acceleration
        b.vel.x += b.acc.x * dt;
        b.vel.y += b.acc.y * dt;
        b.vel.z += b.acc.z * dt;

        // Clamp speed
        float speed2 = b.vel.x * b.vel.x + b.vel.y * b.vel.y + b.vel.z * b.vel.z;
        if (speed2 > maxSpeed * maxSpeed) {
            float invSpeed = maxSpeed / std::sqrt(speed2);
            b.vel.x *= invSpeed;
            b.vel.y *= invSpeed;
            b.vel.z *= invSpeed;
        } else if (speed2 < minSpeed * minSpeed) {
            float invSpeed = minSpeed / std::sqrt(speed2);
            b.vel.x *= invSpeed;
            b.vel.y *= invSpeed;
            b.vel.z *= invSpeed;
        }

        // Integrate velocity
        b.pos.x += b.vel.x * dt;
        b.pos.y += b.vel.y * dt;
        b.pos.z += b.vel.z * dt;

        // Bounce off of walls
        if (bounce) {
            if (b.pos.x < rBoid) {
                b.pos.x = rBoid;
                b.vel.x = -b.vel.x * WALL_BOUNCE;
            } else if (b.pos.x > worldX - rBoid) {
                b.pos.x = worldX - rBoid;
                b.vel.x = -b.vel.x * WALL_BOUNCE;
            }
            if (b.pos.y < rBoid) {
                b.pos.y = rBoid;
                b.vel.y = -b.vel.y * WALL_BOUNCE;
            } else if (b.pos.y > worldY - rBoid) {
                b.pos.y = worldY - rBoid;
                b.vel.y = -b.vel.y * WALL_BOUNCE;
            }
            if (b.pos.z < rBoid) {
                b.pos.z = rBoid;
                b.vel.z = -b.vel.z * WALL_BOUNCE;
            } else if (b.pos.z > worldZ - rBoid) {
                b.pos.z = worldZ - rBoid;
                b.vel.z = -b.vel.z * WALL_BOUNCE;
            }
        } else {
            if (b.pos.x < 0) b.pos.x += worldX;
            else if (b.pos.x >= worldX) b.pos.x -= worldX;
            if (b.pos.y < 0) b.pos.y += worldY;
            else if (b.pos.y >= worldY) b.pos.y -= worldY;
            if (b.pos.z < 0) b.pos.z += worldZ;
            else if (b.pos.z >= worldZ) b.pos.z -= worldZ;
        }

        if (is2D) {
            b.pos.z = 0.0f;
            b.vel.z = 0.0f;
        }
    }
}
