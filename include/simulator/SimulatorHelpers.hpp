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
        std::vector<int>& indices,
        int& count,
        ConfigParameter& target
    );

    // Remove boids from a type group using its index list
    void removeBoids(
        Boids& boids,
        std::vector<int>& indices,
        int& count,
        int howMany
    );

    // Manual spawn actions (UI / user input)
    void spawnPredator(SimState& s, float x, float y);
    void spawnObstacle(SimState& s, float x, float y);

    // Remove boids in radius (used for click delete)
    int deleteAllInRadius(
        SimState& s,
        std::vector<int>& indices,
        int& count,
        float x,
        float y,
        float radius
    );
}
