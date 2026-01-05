#include <random>
#include <vector>
#include <cassert>
#include <algorithm>

#include "simulator/SimulatorHelpers.hpp"

#include "core/Types.hpp"
#include "core/SimState.hpp"


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

// Boid management helpers
void spawnBoid(SimState& s, BoidType type, const Vec3& pos, const Vec3& vel) {
    Boids& b = s.boids;

    if (type == BoidType::Basic) {
        b.posBasic.push_back(pos);
        b.velBasic.push_back(vel);
        b.accBasic.push_back({0,0,0});

        b.targetPointBasic.push_back({
            s.worldX.number() * 0.5f,
            s.worldY.number() * 0.5f,
            s.worldZ.number() * 0.5f
        });
    } else if (type == BoidType::Predator) {
        b.posPredator.push_back(pos);
        b.velPredator.push_back(vel);
        b.accPredator.push_back({0,0,0});

        b.targetPointPredator.push_back({
            s.worldX.number() * 0.5f,
            s.worldY.number() * 0.5f,
            s.worldZ.number() * 0.5f
        });

        b.targetBoidIdxPredator.push_back(-1);
        b.targetBoidDistancePredator.push_back(-1.0f);
        b.targetBoidTypePredator.push_back(BoidType::Empty);

        b.staminaPredator.push_back(100.0f);
        b.restingPredator.push_back(0);
    } else if (type == BoidType::Obstacle) {
        b.posObstacle.push_back(pos);
    } else {
        printf("Warning: Unknown boid type in writeSpawnData(). (type=%d)\n", static_cast<int>(type));
    }
}

void removeLastBoid(Boids& b, BoidType type) {
    if (type == BoidType::Basic) {
        b.posBasic.pop_back();
        b.velBasic.pop_back();
        b.accBasic.pop_back();
        b.targetPointBasic.pop_back();
    } else if (type == BoidType::Predator) {
        b.posPredator.pop_back();
        b.velPredator.pop_back();
        b.accPredator.pop_back();
        b.targetPointPredator.pop_back();

        b.staminaPredator.pop_back();
        b.restingPredator.pop_back();

        b.targetBoidIdxPredator.pop_back();
        b.targetBoidDistancePredator.pop_back();
        b.targetBoidTypePredator.pop_back();
    } else if (type == BoidType::Obstacle) {
        b.posObstacle.pop_back();
    } else {
        printf("Warning: Unknown boid type in removeBoid(). (type=%d)\n", static_cast<int>(type));
    }
}

// Move last boid into specified slot (for deletions)
void moveLastBoidIntoSlot(SimState& s, BoidType type, int slotIdx) {
    Boids& b = s.boids;
    
    if (type == BoidType::Basic) {
        const int lastIdx = b.basicBoidCount - 1;

        if (slotIdx != lastIdx) {
            b.posBasic[slotIdx] = b.posBasic[lastIdx];
            b.velBasic[slotIdx] = b.velBasic[lastIdx];
            b.accBasic[slotIdx] = b.accBasic[lastIdx];
            b.targetPointBasic[slotIdx] = b.targetPointBasic[lastIdx];
        }
    } else if (type == BoidType::Predator) {
        const int lastIdx = b.predatorBoidCount - 1;

        if (slotIdx != lastIdx) {
            b.posPredator[slotIdx] = b.posPredator[lastIdx];
            b.velPredator[slotIdx] = b.velPredator[lastIdx];
            b.accPredator[slotIdx] = b.accPredator[lastIdx];
            b.targetPointPredator[slotIdx] = b.targetPointPredator[lastIdx];

            b.staminaPredator[slotIdx] = b.staminaPredator[lastIdx];
            b.restingPredator[slotIdx] = b.restingPredator[lastIdx];

            b.targetBoidIdxPredator[slotIdx] = b.targetBoidIdxPredator[lastIdx];
            b.targetBoidDistancePredator[slotIdx] = b.targetBoidDistancePredator[lastIdx];
            b.targetBoidTypePredator[slotIdx] = b.targetBoidTypePredator[lastIdx];
        }
    } else if (type == BoidType::Obstacle) {
        const int lastIdx =  b.obstacleBoidCount - 1;

        if (slotIdx != lastIdx) {
            b.posObstacle[slotIdx] = b.posObstacle[lastIdx];
        }

    } else {
        printf("Warning: Unknown boid type in moveLastBoidIntoSlot(). (type=%d)\n", static_cast<int>(type));
    }
}

void spawnBoids(SimState& s, BoidType type, int& count, int howMany, float x, float y, float z) {
    const float wx = s.worldX.number();
    const float wy = s.worldY.number();
    const float wz = s.worldZ.number();
    const float speed = s.initialAxialSpeedRange.number();

    for (int i = 0; i < howMany; ++i) {
        Vec3 pos {
            (x < 0) ? randRange(0, wx) : x,
            (y < 0) ? randRange(0, wy) : y,
            (z < 0) ? randRange(0, wz) : z,
        };

        Vec3 vel {
            randRange(-speed, speed),
            randRange(-speed, speed),
            randRange(-speed, speed)
        };

        if (s.dimensions.string() == "2D") {
            vel.z = pos.z = 0.0f;
        }

        spawnBoid(s, type, pos, vel);

        ++count;
    }
}

void removeBoids(Boids& b, BoidType type, int& count, int howMany) {
    while (howMany-- > 0 && count > 0) {
        removeLastBoid(b, type);
        --count;
    }
}

int deleteBoidsInRadius(SimState& s, BoidType type, int& count, float x, float y, float radius) {
    Boids& b = s.boids;

    std::vector<Vec3>* positions = nullptr;
    if (type == BoidType::Basic) {
        positions = &b.posBasic;
    } else if (type == BoidType::Predator) {
        positions = &b.posPredator;
    } else if (type == BoidType::Obstacle) {
        positions = &b.posObstacle;
    } else {
        printf("Warning: Unknown boid type in deleteBoidsInRadius(). (type=%d)\n", static_cast<int>(type));
        return 0;
    }

    int removed = 0;
    const float r2 = radius * radius;

    for (int i = 0; i < count; ) { 
        const Vec3& p = (*positions)[i];

        const float dx = p.x - x;
        const float dy = p.y - y;

        if (dx*dx + dy*dy <= r2) {
            // Move last boid into this slot
            moveLastBoidIntoSlot(s, type, i);

            // Remove last boid
            removeLastBoid(b, type);

            --count;
            ++removed;
        } else {
            ++i;
        }
    }

    return removed;
}

// Regulation for a specific boid type
void regulateBoidPopulation(SimState& s, BoidType type, int& count, int target) {
    const int delta = target - count;
    const int maxDelta = (int)s.maxBoidPopulationChangeRate.number();

    if (delta > 0)
        spawnBoids(s, type, count, std::min(delta, maxDelta));
    else if (delta < 0)
        removeBoids(s.boids, type, count, std::min(-delta, maxDelta));
}