#include "versions/Parallel.hpp"
#include <cmath>
#include <algorithm>

static constexpr float EPS = 1e-5f;

static inline float sqrLen(const Vec3& v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

static inline Vec3 normalize(const Vec3& v) {
    float l2 = sqrLen(v);
    if (l2 < EPS)
        return {0,0,0};

    float inv = 1.0f / std::sqrt(l2);
    return { v.x*inv, v.y*inv, v.z*inv };
}

void parallelSimulationStep(SimState& simState, const Config&)
{
    const float dt   = 0.016f;
    const bool is2D  = false;

    // ================== STRONG + FAST SWARM ==================

    const float visionRadius      = 50.0f;

    const float alignmentWeight   = 25.4f;
    const float cohesionWeight    = 15.6f;
    const float separationWeight  = 15.5f;

    const float predatorAvoidRadius = 55.0f;
    const float predatorAvoidWeight = 9.5f;

    const float localTargetWeight   = 10.8f;

    // tiny damping, high kinetic energy
    const float drag  = 0.0006f;
    const float noise = 0.16f;

    // ================== HARD SPEED CLAMP (FASTER) ==================

    const float maxSpeed = 150.0f;
    const float minSpeed = 100.0f;

    // ================== WORLD ==================

    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    const float rObstacle = simState.obstacleRadius.number();
    const float rBasic    = simState.basicBoidRadius.number();
    const float rPred     = simState.predatorRadius.number();

    const float vision2        = visionRadius * visionRadius;
    const float predatorAvoid2 = predatorAvoidRadius * predatorAvoidRadius;

    // ================== WALL REPULSION ==================

    auto pushFromWall = [&](float pos, float world, float margin, float k)
    {
        if (pos < margin)
            return (margin - pos) * k;
        if (pos > world - margin)
            return -((pos - (world - margin)) * k);
        return 0.0f;
    };

    const float wallMargin = 65.0f;
    const float wallK      = 5.2f;

    // ================== GLOBAL CENTER PULL ==================

    Vec3 worldCenter {
        worldX * 0.5f,
        worldY * 0.5f,
        worldZ * 0.5f
    };

    const float centerPull = 2.8f;

    // =======================================================

    for (int i = 0; i < (int)simState.boids.size(); ++i)
    {
        Boid& b = simState.boids[i];
        b.acc = {0,0,0};

        if (b.type == BoidType::Obstacle)
            continue;

        const float rBoid =
            (b.type == BoidType::Predator) ? rPred : rBasic;

        // // occasional hard retarget — prevents orbit lock-in
        // if ((rand() % 350) == 0)
        // {
        //     b.target.x = ((float)rand()/RAND_MAX - 0.5f) * worldX;
        //     b.target.y = ((float)rand()/RAND_MAX - 0.5f) * worldY;
        //     b.target.z = ((float)rand()/RAND_MAX - 0.5f) * worldZ;
        // }

        // ================== NEIGHBOURS (STRONG) ==================

        Vec3 alignSum{0,0,0};
        Vec3 cohSum{0,0,0};
        Vec3 sepSum{0,0,0};
        int  ncount = 0;

        for (size_t j : simState.basicBoidIndices)
        {
            if ((int)j == i)
                continue;

            const Boid& o = simState.boids[j];

            Vec3 d {
                o.pos.x - b.pos.x,
                o.pos.y - b.pos.y,
                o.pos.z - b.pos.z
            };

            float d2 = sqrLen(d);
            if (d2 > vision2)
                continue;

            float dlen = std::sqrt(d2);
            if (dlen < 1e-6f)
                continue;

            Vec3 n{ d.x/dlen, d.y/dlen, d.z/dlen };

            alignSum.x += o.vel.x;
            alignSum.y += o.vel.y;
            alignSum.z += o.vel.z;

            cohSum.x += o.pos.x;
            cohSum.y += o.pos.y;
            cohSum.z += o.pos.z;

            sepSum.x -= n.x;
            sepSum.y -= n.y;
            sepSum.z -= n.z;

            ncount++;
        }

        if (ncount > 0)
        {
            float inv = 1.0f / ncount;

            Vec3 avgPos{ cohSum.x*inv, cohSum.y*inv, cohSum.z*inv };
            Vec3 avgVel{ alignSum.x*inv, alignSum.y*inv, alignSum.z*inv };

            Vec3 coh = normalize({
                avgPos.x - b.pos.x,
                avgPos.y - b.pos.y,
                avgPos.z - b.pos.z
            });

            Vec3 ali = normalize({
                avgVel.x - b.vel.x,
                avgVel.y - b.vel.y,
                avgVel.z - b.vel.z
            });

            Vec3 sep = normalize(sepSum);

            b.acc.x += coh.x*cohesionWeight + ali.x*alignmentWeight + sep.x*separationWeight;
            b.acc.y += coh.y*cohesionWeight + ali.y*alignmentWeight + sep.y*separationWeight;
            b.acc.z += coh.z*cohesionWeight + ali.z*alignmentWeight + sep.z*separationWeight;
        }

        // ================== PREDATOR AVOID (VERY STRONG) ==================

        for (size_t pj : simState.predatorBoidIndices)
        {
            const Boid& p = simState.boids[pj];

            Vec3 d {
                p.pos.x - b.pos.x,
                p.pos.y - b.pos.y,
                p.pos.z - b.pos.z
            };

            float d2 = sqrLen(d);
            if (d2 > predatorAvoid2)
                continue;

            Vec3 n = normalize(d);

            b.acc.x -= n.x * predatorAvoidWeight;
            b.acc.y -= n.y * predatorAvoidWeight;
            b.acc.z -= n.z * predatorAvoidWeight;
        }

        // ================== LOCAL TARGET (STRONG) ==================

        Vec3 tdir = normalize({
            b.targetPoint.x - b.pos.x,
            b.targetPoint.y - b.pos.y,
            b.targetPoint.z - b.pos.z
        });

        b.acc.x += tdir.x * localTargetWeight;
        b.acc.y += tdir.y * localTargetWeight;
        b.acc.z += tdir.z * localTargetWeight;

        // ================== WALL PUSH ==================

        b.acc.x += pushFromWall(b.pos.x, worldX, wallMargin, wallK);
        b.acc.y += pushFromWall(b.pos.y, worldY, wallMargin, wallK);
        b.acc.z += pushFromWall(b.pos.z, worldZ, wallMargin, wallK);

        // ================== CENTER PULL ==================

        Vec3 toCenter {
            worldCenter.x - b.pos.x,
            worldCenter.y - b.pos.y,
            worldCenter.z - b.pos.z
        };

        Vec3 cdir = normalize(toCenter);

        b.acc.x += cdir.x * centerPull;
        b.acc.y += cdir.y * centerPull;
        b.acc.z += cdir.z * centerPull;

        // ================== DRAG + NOISE ==================

        b.acc.x += -b.vel.x * drag + ((float)rand()/RAND_MAX - 0.5f) * noise;
        b.acc.y += -b.vel.y * drag + ((float)rand()/RAND_MAX - 0.5f) * noise;
        b.acc.z += -b.vel.z * drag + ((float)rand()/RAND_MAX - 0.5f) * noise;

        // integrate acceleration
        b.vel.x += b.acc.x * dt;
        b.vel.y += b.acc.y * dt;
        b.vel.z += b.acc.z * dt;

        // ================== STRICT SPEED CLAMP (FASTER) ==================

        float s = std::sqrt(std::max(sqrLen(b.vel), 1e-6f));

        float localMinSpeed =
            (b.pos.x < wallMargin || b.pos.x > worldX - wallMargin)
            ? std::max(minSpeed, 14.0f)
            : minSpeed;

        if (s > maxSpeed)
        {
            float k = maxSpeed / s;
            b.vel.x *= k; b.vel.y *= k; b.vel.z *= k;
        }
        else if (s < localMinSpeed)
        {
            float k = localMinSpeed / s;
            b.vel.x *= k; b.vel.y *= k; b.vel.z *= k;
        }

        // integrate velocity
        b.pos.x += b.vel.x * dt;
        b.pos.y += b.vel.y * dt;
        b.pos.z += b.vel.z * dt;

        // clamp inside world
        b.pos.x = std::clamp(b.pos.x, rBoid, worldX - rBoid);
        b.pos.y = std::clamp(b.pos.y, rBoid, worldY - rBoid);
        b.pos.z = std::clamp(b.pos.z, rBoid, worldZ - rBoid);

        if (is2D)
        {
            b.pos.z = 0.0f;
            b.vel.z = 0.0f;
        }
    }
}
