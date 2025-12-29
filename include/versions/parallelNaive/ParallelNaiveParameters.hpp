#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"


struct ParallelNaiveParameters {
    // Reference to boid array
    std::vector<Boid>& boids;

    // TODO: Add other parameters as needed

    // Constructor
    ParallelNaiveParameters(SimState& s, const Config& c);
};
