#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"


struct SequentialParameters {
    // Reference to boid array
    std::vector<Boid>& boids;

    // TODO: Add other parameters as needed

    // Constructor
    SequentialParameters(SimState& s, const Config& c);
};
