#include <algorithm>

#include "versions/sequential/SequentialParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


SequentialParameters::SequentialParameters(SimState& s, const Config& c) 
    : boids(s.boids) {
    // TODO: Initialize other parameters as needed
}
