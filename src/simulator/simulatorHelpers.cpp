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
    size_t allocateBoidSlot(std::vector<Boid>& boids, std::vector<size_t>& freeBoidIndices) {
        if (!freeBoidIndices.empty()) {
            size_t idx = freeBoidIndices.back();
            freeBoidIndices.pop_back();
            return idx;
        }
        boids.emplace_back();
        return boids.size() - 1;
    }

    void freeBoidSlot(std::vector<Boid>& boids, std::vector<size_t>& freeBoidIndices, size_t idx) {
        assert(idx < boids.size());

        boids[idx].type = BoidType::Empty;
        freeBoidIndices.push_back(idx);
    }

    void spawnBoids(std::vector<Boid>& boids, std::vector<size_t>& freeBoidIndices,
                    float wx, float wy, float wz, float speedRange,
                    BoidType type, std::vector<size_t>& indices, uint64_t& count, int howMany, 
                    ConfigParameter& radius, ConfigParameter& color, std::string dimensions) {
        for (int i = 0; i < howMany; ++i) {
            size_t idx = allocateBoidSlot(boids, freeBoidIndices);
            Boid& b = boids[idx];

            b.type = type;
            b.pos  = { randRange(0,wx), randRange(0,wy), randRange(0,wz) };
            b.vel  = { randRange(-speedRange,speedRange), randRange(-speedRange,speedRange), randRange(-speedRange,speedRange) };
            b.acc  = {0,0,0};

            b.radius = radius.number();
            b.color  = color.string();

            if (dimensions == "2D") {
                b.pos.z = 0;
                b.vel.z = 0;
            }

            indices.push_back(idx);
            ++count;
        }
    }

    void removeBoids(std::vector<Boid>& boids, std::vector<size_t>& freeBoidIndices, std::vector<size_t>& indices, uint64_t& count, int howMany) {
        while (howMany-- > 0 && !indices.empty()) {
            size_t idx = indices.back();
            indices.pop_back();
            freeBoidSlot(boids, freeBoidIndices, idx);
            --count;
        }
    }

    int deleteAllInRadius(SimState& s, std::vector<size_t>& indices, uint64_t& count, float x, float y, float radius) {
        int removed = 0;

        for (size_t i = 0; i < indices.size(); ) {
            const size_t idx = indices[i];
            const Boid& b = s.boids[idx];

            if (dist2(b.pos.x, b.pos.y, x, y) <= radius * radius) {
                freeBoidSlot(s.boids, s.freeBoidIndices, idx);
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

    // Spawn helpers
    void spawnPredator(SimState& s, float x, float y) {
        if (!canIncreaseTarget(s.predatorBoidCountTarget))
            return;

        size_t idx = allocateBoidSlot(s.boids, s.freeBoidIndices);
        Boid& b = s.boids[idx];

        b.type   = BoidType::Predator;
        b.pos    = { x, y, 0.0f };
        b.vel    = {};
        b.acc    = {};
        b.radius = s.predatorRadius.number();
        b.color  = s.predatorBoidColor.string();

        s.predatorBoidIndices.push_back(idx);
        ++s.predatorBoidCount;
        s.predatorBoidCountTarget.number() += 1.0f;
    }

    void spawnObstacle(SimState& s, float x, float y) {
        size_t idx = allocateBoidSlot(s.boids, s.freeBoidIndices);
        Boid& b = s.boids[idx];

        b.type   = BoidType::Obstacle;
        b.pos    = { x, y, 0.0f };
        b.vel    = {};
        b.acc    = {};
        b.radius = s.obstacleRadius.number();
        b.color  = s.obstacleBoidColor.string();

        s.obstacleBoidIndices.push_back(idx);
        ++s.obstacleBoidCount;
    }

    // Regulation for a specific boid type
    void regulateType(SimState& s, BoidType type, std::vector<size_t>& indices, uint64_t& count, 
                      ConfigParameter& target, ConfigParameter& radius, ConfigParameter& color) {
        const int tgt   = (int)target.number();
        const int cur   = (int)count;
        const int delta = tgt - cur;
        const int maxDelta  = (int)s.maxBoidPopulationChangeRate.number();
        std::vector<Boid>& boids = s.boids;
        std::vector<size_t>& freeBoidIndices = s.freeBoidIndices;
        const float worldX = s.worldX.number();
        const float worldY = s.worldY.number();
        const float worldZ = s.worldZ.number();
        const float speedRange = s.initialAxialSpeedRange.number();
        const std::string dimensions = s.dimensions.string();

        if (delta > 0) {
            spawnBoids(boids, freeBoidIndices, worldX, worldY, worldZ, speedRange, 
                       type, indices, count, std::min(delta, maxDelta), radius, color, dimensions);
        } else if (delta < 0) {
            removeBoids(boids, freeBoidIndices, indices, count, std::min(-delta, maxDelta));
        }
    }
}