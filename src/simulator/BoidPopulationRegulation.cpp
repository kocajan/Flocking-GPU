#include "core/SimState.hpp"
#include "boids/Boid.hpp"

#include <algorithm>
#include <random>

namespace {

// ------------------------------------------------------------
// Tuning constants (hardcoded for now)
// ------------------------------------------------------------
constexpr int MAX_SPAWN_PER_FRAME   = 20;
constexpr int MAX_DESPAWN_PER_FRAME = 20;

// ------------------------------------------------------------
// Random helpers (CPU only, simple)
// ------------------------------------------------------------
float rand01() {
    static std::mt19937 rng{ std::random_device{}() };
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return dist(rng);
}

float randRange(float a, float b) {
    return a + (b - a) * rand01();
}

// ------------------------------------------------------------
// Bird spawn / removal helpers
// ------------------------------------------------------------
void spawnBirds(SimState& simState, int count) {
    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    for (int i = 0; i < count; ++i) {
        Boid b{};
        b.type = BoidType::Bird;

        // random position in world bounds
        b.pos = {
            randRange(0.0f, worldX),
            randRange(0.0f, worldY),
            randRange(0.0f, worldZ)
        };

        // random initial velocity
        b.vel = {
            randRange(-1.0f, 1.0f),
            randRange(-1.0f, 1.0f),
            randRange(-1.0f, 1.0f)
        };

        b.acc = {0.0f, 0.0f, 0.0f};

        // Give a default color (white)
        b.color = {1.0f, 1.0f, 1.0f, 1.0f};

        // default bird parameters (temporary hardcoded)
        b.radius = 2.0f;
        b.maxSpeed = 50.0f;
        b.maxForce = 10.0f;

        b.alignmentWeight  = 1.0f;
        b.cohesionWeight   = 1.0f;
        b.separationWeight = 1.2f;

        // If running in 2D, lock Z
        if (simState.dimensions.string() == "2D") {
            b.pos.z = 0.0f;
            b.vel.z = 0.0f;
        }

        simState.boids.push_back(b);
    }
}

void removeBirds(SimState& simState, int count) {
    for (int i = 0; i < count; ++i) {
        for (size_t j = 0; j < simState.boids.size(); ++j) {
            if (simState.boids[j].type == BoidType::Bird) {
                // swap-with-last removal
                simState.boids[j] = simState.boids.back();
                simState.boids.pop_back();
                break;
            }
        }
    }
}

} // anonymous namespace

// ------------------------------------------------------------
// Public entry point
// ------------------------------------------------------------
void regulateBoidPopulation(SimState& simState) {
    const int target = simState.boidCount.number();

    int current = 0;
    for (const Boid& b : simState.boids) {
        if (b.type == BoidType::Bird)
            current++;
    }

    const int delta = target - current;

    if (delta > 0) {
        const int spawn = std::min(delta, MAX_SPAWN_PER_FRAME);
        spawnBirds(simState, spawn);
    }
    else if (delta < 0) {
        const int remove = std::min(-delta, MAX_DESPAWN_PER_FRAME);
        removeBirds(simState, remove);
    }
}
