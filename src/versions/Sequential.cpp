#include "versions/Sequential.hpp"

#include <cmath>


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

void sequentialSimulationStep(SimState& simState, const Config& simConfig) {
    const float dt   = simState.dt.number();
    const bool  is2D = (simState.dimensions.string() == "2D");
    const bool  bounce = simConfig.binary("bounce");

    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    // Boids visual + protected radii
    const float visualRange     = simConfig.number("vision");

    // Rule weights (Boids Lab terms)
    const float centeringFactor = simConfig.number("cohesion");     // Cohesion
    const float matchingFactor  = simConfig.number("alignment");    // Alignment
    const float avoidFactor     = simConfig.number("separation") * 100.0f;   // Separation

    // Dynamics
    const float maxSpeed = simConfig.number("max_speed");
    const float minSpeed = simConfig.number("min_speed");
    const float drag     = simConfig.number("drag");
    const float noise    = simConfig.number("noise");

    // Walls
    constexpr float WALL_BOUNCE = 0.6f;

    // Obstacles
    std::vector<size_t> obstacleBoidIndices = simState.obstacleBoidIndices;
    const float rObstacle = simState.obstacleRadius.number();
    const float rBasic    = simState.basicBoidRadius.number();
    const float rPred     = simState.predatorRadius.number();
    constexpr float BOUNCE_FACTOR = 0.6f;

    // Zero-length constant
    const float EPS = 1e-5f;

    // Local target weight
    const float localTargetWeight = 0.01f;

    for (int currentBoidIdx = 0; currentBoidIdx < simState.boids.size(); ++currentBoidIdx) {
        Boid& b = simState.boids[currentBoidIdx];
        b.acc = {0,0,0};

        if (b.type == BoidType::Obstacle)
            continue;

        const float rBoid = (b.type == BoidType::Predator) ? rPred : rBasic;
        const float rBoid2 = rBoid * rBoid;

        // ============================================================
        // Boids Lab rule accumulators
        // ============================================================
        Vec3 closeDelta {0,0,0};     // separation (protected range)
        Vec3 centerSum  {0,0,0};     // cohesion
        Vec3 velSum     {0,0,0};     // alignment
        int  neighborCount = 0;

        const float visual2    = visualRange * visualRange;

        for (size_t otherIdx : simState.basicBoidIndices) {
            if (otherIdx == currentBoidIdx) continue;

            const Boid& o = simState.boids[otherIdx];

            Vec3 d = {
                o.pos.x - b.pos.x,
                o.pos.y - b.pos.y,
                o.pos.z - b.pos.z
            };

            float d2 = sqrLen(d);
            if (d2 > visual2)
                continue;

            float dist = std::sqrt(d2);
            if (dist < EPS)
                dist = EPS;

            neighborCount++;

            // Separation — only inside protected range
            if (d2 < rBoid2 * 4.0f) {
                float invd = 1.0f / dist;
                closeDelta.x -= (o.pos.x - b.pos.x) * invd;
                closeDelta.y -= (o.pos.y - b.pos.y) * invd;
                closeDelta.z -= (o.pos.z - b.pos.z) * invd;
            } else if (d2 >= rBoid2 * 16.0f) {
            // Cohesion - full vision range but outside protected range
                centerSum.x += o.pos.x;
                centerSum.y += o.pos.y;
                centerSum.z += o.pos.z;    
            }

            // Alignment - full vision range
            velSum.x += o.vel.x;
            velSum.y += o.vel.y;
            velSum.z += o.vel.z;
        }

        if (neighborCount > 0) {
            float invN = 1.0f / neighborCount;

            // Cohesion → steer toward average position
            Vec3 cohesion = {
                (centerSum.x * invN - b.pos.x) * centeringFactor,
                (centerSum.y * invN - b.pos.y) * centeringFactor,
                (centerSum.z * invN - b.pos.z) * centeringFactor
            };
            b.acc.x += cohesion.x;
            b.acc.y += cohesion.y;
            b.acc.z += cohesion.z;

            // Alignment → steer toward average velocity
            Vec3 alignment = {
                (velSum.x * invN - b.vel.x) * matchingFactor,
                (velSum.y * invN - b.vel.y) * matchingFactor,
                (velSum.z * invN - b.vel.z) * matchingFactor
            };
            b.acc.x += alignment.x;
            b.acc.y += alignment.y;
            b.acc.z += alignment.z;

            // Separation → avoid crowding
            b.acc.x += closeDelta.x * avoidFactor;
            b.acc.y += closeDelta.y * avoidFactor;
            b.acc.z += closeDelta.z * avoidFactor;
        }

        // ============================================================
        // Local target attraction
        // - The further from the target, the stronger the pull
        // ============================================================
        Vec3 toTarget = {
            b.targetPoint.x - b.pos.x,
            b.targetPoint.y - b.pos.y,
            b.targetPoint.z - b.pos.z
        };
        if (is2D)
            toTarget.z = 0.0f;
        Vec3 toTargetN = normalize(toTarget, EPS);
        float toTargetLen = std::sqrt(sqrLen(toTarget));
        b.acc.x += toTargetN.x * (toTargetLen * toTargetLen) * localTargetWeight;
        b.acc.y += toTargetN.y * (toTargetLen * toTargetLen) * localTargetWeight;
        b.acc.z += toTargetN.z * (toTargetLen * toTargetLen) * localTargetWeight;

        // ============================================================
        // Mouse interaction — unchanged
        // ============================================================
        const Interaction& inter = simState.interaction;
        if (inter.type != InteractionType::Empty) {
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
            if (forceMag > 300.0f) forceMag = 300.0f;

            if (inter.type == InteractionType::Attract) {
                b.acc.x -= nx * forceMag;
                b.acc.y -= ny * forceMag;
                b.acc.z -= nz * forceMag;
            } else {
                b.acc.x += nx * forceMag;
                b.acc.y += ny * forceMag;
                b.acc.z += nz * forceMag;
            }
        }

        // ============================================================
        // Obstacles — kept as in your original version
        // ============================================================
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
                b.pos.x += n.x * (-dist + EPS);
                b.pos.y += n.y * (-dist + EPS);
                b.pos.z += n.z * (-dist + EPS);
                float vDotN = b.vel.x*n.x + b.vel.y*n.y + b.vel.z*n.z;
                b.vel.x -= (1.0f + BOUNCE_FACTOR) * vDotN * n.x;
                b.vel.y -= (1.0f + BOUNCE_FACTOR) * vDotN * n.y;
                b.vel.z -= (1.0f + BOUNCE_FACTOR) * vDotN * n.z;
            } else {
                float forceMag = 1000.0f * std::exp(-dist / 5.0f);
                Vec3 n = normalize(diff);
                b.acc.x += n.x * forceMag;
                b.acc.y += n.y * forceMag;
                b.acc.z += n.z * forceMag;
            }
        }

        // Drag + noise
        b.acc.x += -b.vel.x * drag;
        b.acc.y += -b.vel.y * drag;
        b.acc.z += -b.vel.z * drag;

        b.acc.x += ((float)rand() / RAND_MAX - 0.5f) * noise;
        b.acc.y += ((float)rand() / RAND_MAX - 0.5f) * noise;
        b.acc.z += ((float)rand() / RAND_MAX - 0.5f) * noise;

        // Integrate
        b.vel.x += b.acc.x * dt;
        b.vel.y += b.acc.y * dt;
        b.vel.z += b.acc.z * dt;

        // Speed limits (Boids Lab style)
        float speed = std::sqrt(sqrLen(b.vel));
        if (speed > maxSpeed) {
            float s = maxSpeed / speed;
            b.vel.x *= s; b.vel.y *= s; b.vel.z *= s;
        } else if (speed < minSpeed) {
            float s = minSpeed / (speed + EPS);
            b.vel.x *= s; b.vel.y *= s; b.vel.z *= s;
        }

        b.pos.x += b.vel.x * dt;
        b.pos.y += b.vel.y * dt;
        b.pos.z += b.vel.z * dt;

        // Wall bounce / wrap — preserved
        if (bounce) {
            if (b.pos.x < rBoid) { b.pos.x = rBoid; b.vel.x = -b.vel.x * WALL_BOUNCE; }
            else if (b.pos.x > worldX - rBoid) { b.pos.x = worldX - rBoid; b.vel.x = -b.vel.x * WALL_BOUNCE; }

            if (b.pos.y < rBoid) { b.pos.y = rBoid; b.vel.y = -b.vel.y * WALL_BOUNCE; }
            else if (b.pos.y > worldY - rBoid) { b.pos.y = worldY - rBoid; b.vel.y = -b.vel.y * WALL_BOUNCE; }

            if (!is2D) {
                if (b.pos.z < rBoid) { b.pos.z = rBoid; b.vel.z = -b.vel.z * WALL_BOUNCE; }
                else if (b.pos.z > worldZ - rBoid) { b.pos.z = worldZ - rBoid; b.vel.z = -b.vel.z * WALL_BOUNCE; }
            }
        } else {
            if (b.pos.x < 0) b.pos.x += worldX; else if (b.pos.x >= worldX) b.pos.x -= worldX;
            if (b.pos.y < 0) b.pos.y += worldY; else if (b.pos.y >= worldY) b.pos.y -= worldY;
            if (!is2D) {
                if (b.pos.z < 0) b.pos.z += worldZ; else if (b.pos.z >= worldZ) b.pos.z -= worldZ;
            }
        }

        if (is2D) {
            b.pos.z = 0.0f;
            b.vel.z = 0.0f;
        }
    }
}
