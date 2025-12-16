#include "core/SimState.hpp"
#include "boids/Boid.hpp"

#include <algorithm>
#include <random>
#include <cassert>

namespace {

// ============================================================
// Random helpers
// ============================================================

float rand01() {
    static std::mt19937 rng{ std::random_device{}() };
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return dist(rng);
}

float randRange(float a, float b) {
    return a + (b - a) * rand01();
}

// ============================================================
// Color parsing
// ============================================================

Color parseColorString(const std::string& s) {
    if (s == "White")  return {1,1,1,1};
    if (s == "Black")  return {0,0,0,1};
    if (s == "Red")    return {1,0,0,1};
    if (s == "Green")  return {0,1,0,1};
    if (s == "Blue")   return {0,0,1,1};
    return {0.5f,0.5f,0.5f,1};
}

// ============================================================
// Free-list allocation
// ============================================================

size_t allocateBoidSlot(SimState& s) {
    if (!s.freeBoidIndices.empty()) {
        size_t idx = s.freeBoidIndices.back();
        s.freeBoidIndices.pop_back();
        return idx;
    }
    s.boids.emplace_back();
    return s.boids.size() - 1;
}

void freeBoidSlot(SimState& s, size_t idx) {
#ifndef NDEBUG
    assert(idx < s.boids.size());
#endif
    s.boids[idx].type = BoidType::Custom;
    s.freeBoidIndices.push_back(idx);
}

// ============================================================
// Spawn / Remove (GENERIC)
// ============================================================

void spawnBoids(
    SimState& s,
    BoidType type,
    std::vector<size_t>& indices,
    uint64_t& count,
    int howMany,
    ConfigParameter& radius,
    ConfigParameter& color
) {
    const float wx = s.worldX.number();
    const float wy = s.worldY.number();
    const float wz = s.worldZ.number();

    for (int i = 0; i < howMany; ++i) {
        size_t idx = allocateBoidSlot(s);
        Boid& b = s.boids[idx];

        b.type = type;
        b.pos  = { randRange(0,wx), randRange(0,wy), randRange(0,wz) };
        b.vel  = { randRange(-100,100), randRange(-100,100), randRange(-100,100) };
        b.acc  = {0,0,0};

        b.radius = radius.number();
        b.color  = parseColorString(color.string());

        if (s.dimensions.string() == "2D") {
            b.pos.z = 0;
            b.vel.z = 0;
        }

        indices.push_back(idx);
        ++count;
    }
}

void removeBoids(
    SimState& s,
    std::vector<size_t>& indices,
    uint64_t& count,
    int howMany
) {
    while (howMany-- > 0 && !indices.empty()) {
        size_t idx = indices.back();
        indices.pop_back();
        freeBoidSlot(s, idx);
        --count;
    }
}

// ============================================================
// Regulation
// ============================================================

void regulateType(
    SimState& s,
    BoidType type,
    std::vector<size_t>& indices,
    uint64_t& count,
    ConfigParameter& target,
    ConfigParameter& radius,
    ConfigParameter& color
) {
    const int tgt   = (int)target.number();
    const int cur   = (int)count;
    const int delta = tgt - cur;
    const int maxΔ  = (int)s.maxBoidPopulationChangeRate.number();

    if (delta > 0) {
        spawnBoids(s, type, indices, count, std::min(delta, maxΔ), radius, color);
    } else if (delta < 0) {
        removeBoids(s, indices, count, std::min(-delta, maxΔ));
    }
}

} // namespace

// ============================================================
// Public entry point
// ============================================================

void regulateBoidPopulation(SimState& s) {
    regulateType(
        s,
        BoidType::Basic,
        s.basicBoidIndices,
        s.basicBoidCount,
        s.basicBoidCountTarget,
        s.basicBoidRadius,
        s.basicBoidColor
    );

    regulateType(
        s,
        BoidType::Predator,
        s.predatorBoidIndices,
        s.predatorBoidCount,
        s.predatorBoidCountTarget,
        s.predatorRadius,
        s.predatorBoidColor
    );
}
