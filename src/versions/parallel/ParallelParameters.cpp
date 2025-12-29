#include <algorithm>

#include "versions/parallel/ParallelParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


ParallelParameters::ParallelParameters(SimState& s, const Config& c) 
    : boids(s.boids) {
    // TODO: Initialize other parameters as needed
}
