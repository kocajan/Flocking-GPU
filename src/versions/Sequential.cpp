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

void sequentialSimulationStep(SimState& simState, const SimConfig& simConfig) {
    const float dt   = simState.dt.number();
    const bool  is2D = (simState.dimensions.string() == "2D");
    const bool bounce = simConfig.binary("bounce");

    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    const float maxSpeed = simConfig.number("max_speed");

    // Get boid indices
    std::vector<size_t> obstacleBoidIndices = simState.obstacleBoidIndices;
    std::vector<size_t> predatorBoidIndices = simState.predatorBoidIndices;

    // radii
    const float rObstacle = simState.obstacleRadius.number();
    const float rBasic = simState.basicBoidRadius.number();
    const float rPred = simState.predatorRadius.number();

    // walls
    constexpr float WALL_FORCE     = 2000.0f;
    constexpr float WALL_SOFT_DIST = 5.0f;
    constexpr float WALL_BOUNCE    = 0.6f;

    // obstacle bounce
    constexpr float BOUNCE_FACTOR = 0.6f;

    for (Boid& b : simState.boids) {
        b.acc = {0,0,0};

        if (b.type == BoidType::Obstacle)
            continue;

        float rBoid = (b.type == BoidType::Predator) ? rPred : rBasic;

        // Apply interaction from mouse
        const Interaction& inter = simState.interaction;
        const InteractionType interType = inter.type;
        if (interType != InteractionType::Empty) {
            // Vector from boid to interaction point
            float dx = b.pos.x - inter.point.x;
            float dy = b.pos.y - inter.point.y;
            float dz = b.pos.z - inter.point.z;

            float dist2 = dx*dx + dy*dy + dz*dz;
            if (dist2 < 1e-6f) dist2 = 1e-6f; // prevent div by zero

            float invDist = 1.0f / std::sqrt(dist2);

            // Normalized direction
            float nx = dx * invDist;
            float ny = dy * invDist;
            float nz = dz * invDist;

            // Inverse-square force (matches JS exactly)
            float forceMag = 10000000.0f / dist2;

            // Clamp (equivalent to .max(g.mouseForce))
            if (forceMag > 100.0f)
                forceMag = 100.0f;

            if (interType == InteractionType::Attract) {
                // Pull toward phantom
                b.acc.x -= nx * forceMag;
                b.acc.y -= ny * forceMag;
                b.acc.z -= nz * forceMag;
            }
            else if (interType == InteractionType::Repel) {
                // Push away from phantom
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

            // If they are overlapping, let the boid bounce off
            if (dist < 0.0f) {
                Vec3 n = normalize(toBoid);
                // Simple elastic collision response
                b.pos.x += n.x * (-dist + EPS);
                b.pos.y += n.y * (-dist + EPS);
                b.pos.z += n.z * (-dist + EPS);
                float vDotN = b.vel.x * n.x + b.vel.y * n.y + b.vel.z * n.z;
                b.vel.x -= (1.0f + BOUNCE_FACTOR) * vDotN * n.x;
                b.vel.y -= (1.0f + BOUNCE_FACTOR) * vDotN * n.y;
                b.vel.z -= (1.0f + BOUNCE_FACTOR) * vDotN * n.z;
            } else {
                // Otherwise, apply repulsive force (grows to infinity as we get closer - exponential)
                float forceMag = 1000.0f * std::exp(-dist / 5.0f);
                Vec3 n = normalize(toBoid);
                b.acc.x += n.x * forceMag;
                b.acc.y += n.y * forceMag;
                b.acc.z += n.z * forceMag;
            }
        }

        // Integrate acceleration
        b.vel.x += b.acc.x * dt;
        b.vel.y += b.acc.y * dt;
        b.vel.z += b.acc.z * dt;

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
            // wrap around
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