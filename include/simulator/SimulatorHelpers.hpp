#pragma once

#include <vector>
#include <cstdint>

#include "core/Types.hpp"
#include "core/SimState.hpp"

namespace simulator {
    void regulateType(
        SimState& s,
        BoidType type,
        std::vector<size_t>& indices,
        uint64_t& count,
        ConfigParameter& target,
        ConfigParameter& radius,
        ConfigParameter& color
    );

    void removeBoids(
        std::vector<Boid>& boids,
        std::vector<size_t>& freeBoidIndices,
        std::vector<size_t>& indices,
        uint64_t& count,
        int howMany
    );

    void spawnPredator(SimState& s, float x, float y);

    void spawnObstacle(SimState& s, float x, float y);

    int deleteAllInRadius(
        SimState& s,
        std::vector<size_t>& indices,
        uint64_t& count,
        float x, float y,
        float radius
    );
}
