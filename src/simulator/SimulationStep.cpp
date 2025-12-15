#include <cmath>

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"
#include "boids/Boid.hpp"

void simulationStep(SimState& simState, const SimConfig& /*simConfig*/) {
    const float dt = simState.dt.number();

    // --------------------------------------------------------
    // 1. Reset accelerations (non-obstacles)
    // --------------------------------------------------------
    for (Boid& b : simState.boids) {
        if (b.type != BoidType::Obstacle) {
            b.acc = {0.0f, 0.0f, 0.0f};
        }
    }

    // --------------------------------------------------------
    // 2. Integrate velocity (acc -> vel)
    // --------------------------------------------------------
    for (Boid& b : simState.boids) {
        if (b.type == BoidType::Obstacle)
            continue;

        b.vel.x += b.acc.x * dt;
        b.vel.y += b.acc.y * dt;
        b.vel.z += b.acc.z * dt;

        // Clamp speed (simple magnitude clamp)
        const float v2 =
            b.vel.x * b.vel.x +
            b.vel.y * b.vel.y +
            b.vel.z * b.vel.z;

        const float maxSpeed2 = b.maxSpeed * b.maxSpeed;
        if (v2 > maxSpeed2 && v2 > 0.0f) {
            const float invLen = b.maxSpeed / std::sqrt(v2);
            b.vel.x *= invLen;
            b.vel.y *= invLen;
            b.vel.z *= invLen;
        }
    }

    // --------------------------------------------------------
    // 3. Integrate position (vel -> pos)
    // --------------------------------------------------------
    for (Boid& b : simState.boids) {
        if (b.type == BoidType::Obstacle)
            continue;

        b.pos.x += b.vel.x * dt;
        b.pos.y += b.vel.y * dt;
        b.pos.z += b.vel.z * dt;

        // Lock to 2D if needed
        if (simState.dimensions.string() == "2D") {
            b.pos.z = 0.0f;
            b.vel.z = 0.0f;
        }
    }
}
