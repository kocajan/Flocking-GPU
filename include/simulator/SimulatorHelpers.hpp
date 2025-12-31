#pragma once

#include <vector>
#include <cstdint>

#include "core/Types.hpp"
#include "core/SimState.hpp"


namespace simulator {

    // Regulate population for a single boid type
    void regulateType(
        SimState& s,
        BoidType type,
        std::vector<size_t>& indices,
        uint64_t& count,
        ConfigParameter& target
    );

    // Remove boids from a type group using its index list
    void removeBoids(
        Boids& boids,
        std::vector<size_t>& indices,
        uint64_t& count,
        int howMany
    );

    // Manual spawn actions (UI / user input)
    void spawnPredator(SimState& s, float x, float y);
    void spawnObstacle(SimState& s, float x, float y);

    // Remove boids in radius (used for click delete)
    int deleteAllInRadius(
        SimState& s,
        std::vector<size_t>& indices,
        uint64_t& count,
        float x,
        float y,
        float radius
    );
}
