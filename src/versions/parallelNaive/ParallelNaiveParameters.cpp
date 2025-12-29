#include <algorithm>

#include "versions/parallelNaive/ParallelNaiveParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


ParallelNaiveParameters::ParallelNaiveParameters(SimState& s, const Config& c) 
    : boids(s.boids) {
    // TODO: Initialize other parameters as needed
}
