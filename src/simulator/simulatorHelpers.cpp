#include <random>
#include <vector>
#include <cassert>
#include <algorithm>

#include "simulator/SimulatorHelpers.hpp"

#include "core/Types.hpp"
#include "core/SimState.hpp"


namespace simulator{
    // Random utilities
    float rand01() {
        static std::mt19937 rng{ std::random_device{}() };
        static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        return dist(rng);
    }

    float randRange(float a, float b) {
        return a + (b - a) * rand01();
    }

    // Distance squared helper
    static inline float dist2(float x1, float y1, float x2, float y2) {
        const float dx = x1 - x2;
        const float dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    // Max-limit helper
    static bool canIncreaseTarget(const ConfigParameter& p) {
        const auto& r = std::get<NumberRange>(p.range);
        return p.number() < r.max;
    }

    // Free and allocate boid slots
    int allocateBoidSlot(Boids& b) {
        if (!b.freeBoidIndices.empty()) {
            int idx = b.freeBoidIndices.back();
            b.freeBoidIndices.pop_back();
            return idx;
        }

        int idx = b.allBoidCount;
        b.resize(b.allBoidCount + 1);
        return idx;
    }

    void freeBoidSlot(Boids& b, int idx) {
        assert(idx < b.allBoidCount);
        b.type[idx] = static_cast<uint8_t>(BoidType::Empty);
        b.freeBoidIndices.push_back(idx);
    }

    void writeSpawnData(SimState& s, int idx, BoidType type, const Vec3& pos, const Vec3& vel) {
        Boids& b = s.boids;

        b.type[idx] = static_cast<uint8_t>(type);

        b.pos[idx] = pos;
        b.vel[idx] = vel;
        b.acc[idx] = {0,0,0};

        b.targetPoint[idx] = {
            s.worldX.number() * 0.5f,
            s.worldY.number() * 0.5f,
            s.worldZ.number() * 0.5f
        };

        b.targetBoidIdx[idx] = -1;
        b.targetBoidDistance[idx] = -1.0f;

        b.stamina[idx] = 100.0f;
        b.resting[idx] = 0;
    }

    void spawnBoids(SimState& s, BoidType type, std::vector<int>& indices, int& count, int howMany) {
        Boids& b = s.boids;

        const float wx = s.worldX.number();
        const float wy = s.worldY.number();
        const float wz = s.worldZ.number();
        const float speed = s.initialAxialSpeedRange.number();

        for (int i = 0; i < howMany; ++i) {
            int idx = allocateBoidSlot(b);

            Vec3 pos {
                randRange(0, wx),
                randRange(0, wy),
                randRange(0, wz)
            };

            Vec3 vel {
                randRange(-speed, speed),
                randRange(-speed, speed),
                randRange(-speed, speed)
            };

            if (s.dimensions.string() == "2D") {
                vel.z = pos.z = 0.0f;
            }

            writeSpawnData(s, idx, type, pos, vel);

            indices.push_back(idx);
            ++count;
        }
    }

    void removeBoids(Boids& b, std::vector<int>& indices, int& count, int howMany) {
        while (howMany-- > 0 && !indices.empty()) {
            int idx = indices.back();
            indices.pop_back();

            freeBoidSlot(b, idx);
            --count;
        }
    }

    int deleteAllInRadius(SimState& s, std::vector<int>& indices, int& count, float x, float y, float radius) {
        int removed = 0;
        Boids& b = s.boids;

        const float r2 = radius * radius;

        for (int i = 0; i < (int)indices.size(); ++i) { 
            int idx = indices[i];
            const Vec3& p = b.pos[idx];

            const float dx = p.x - x;
            const float dy = p.y - y;

            if (dx*dx + dy*dy <= r2) {
                freeBoidSlot(b, idx);

                indices[i] = indices.back();
                indices.pop_back();

                --count;
                ++removed;
            } else {
                ++i;
            }
        }

        return removed;
    }

    // Spawn helpers (for interactive spawning)
    void spawnPredator(SimState& s, float x, float y) {
        if (!canIncreaseTarget(s.predatorBoidCountTarget))
            return;

        Boids& b = s.boids;

        int idx = allocateBoidSlot(b);

        float speed = s.initialAxialSpeedRange.number();
        float worldZ = s.worldZ.number();

        Vec3 pos { x, y, randRange(0, worldZ) };
        Vec3 vel {
            randRange(-speed, speed),
            randRange(-speed, speed),
            randRange(-speed, speed)
        };

        if (s.dimensions.string() == "2D")
            pos.z = 0.0f;

        writeSpawnData(s, idx, BoidType::Predator, pos, vel);

        b.predatorBoidIndices.push_back(idx);
        ++b.predatorBoidCount;

        // keep UI target in sync with manual spawn
        s.predatorBoidCountTarget.number() += 1.0f;
    }

    void spawnObstacle(SimState& s, float x, float y) {
        Boids& b = s.boids;

        int idx = allocateBoidSlot(b);

        Vec3 pos { x, y, 0.0f };
        Vec3 vel { 0.0f, 0.0f, 0.0f };

        if (s.dimensions.string() == "2D")
            pos.z = 0.0f;

        writeSpawnData(s, idx, BoidType::Obstacle, pos, vel);

        b.obstacleBoidIndices.push_back(idx);
        ++b.obstacleBoidCount;
    }

    // Regulation for a specific boid type
    void regulateType(SimState& s, BoidType type, std::vector<int>& indices, int& count, ConfigParameter& target) {
        const int tgt = (int)target.number();
        const int cur = count;

        const int delta = tgt - cur;
        const int maxDelta = (int)s.maxBoidPopulationChangeRate.number();

        if (delta > 0)
            spawnBoids(s, type, indices, count, std::min(delta, maxDelta));
        else if (delta < 0)
            removeBoids(s.boids, indices, count, std::min(-delta, maxDelta));
    }
}