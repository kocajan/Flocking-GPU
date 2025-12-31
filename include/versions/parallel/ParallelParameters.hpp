#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"


struct ParallelParameters {
    // Reference to boid array
    Boids& boids;

    // TODO: Add other parameters as needed

    // Constructor
    ParallelParameters(SimState& s, const Config& c);
};
