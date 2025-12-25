#include "core/SimState.hpp"
#include "config/SimConfig.hpp"
#include "boids/Boid.hpp"

#include "versions/SequentialNaive.hpp"
#include "versions/Sequential.hpp"
#include "versions/ParallelNaive.hpp"
#include "versions/Parallel.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>


void simulationStep(SimState& simState, const SimConfig& simConfig) {
    // Get the version
    const std::string& version = simState.version.string();
    if (version == "sequentialNaive") {
        sequentialNaiveSimulationStep(simState, simConfig);
    } else if (version == "sequential") {
        sequentialSimulationStep(simState, simConfig);
    } else if (version == "parallelNaive") {
        parallelNaiveSimulationStep(simState, simConfig);
    } else if (version == "parallel") {
        parallelSimulationStep(simState, simConfig);
    } else {
        // Throw error for unknown version
        throw std::runtime_error("Unknown simulation version: " + version);        
    }
}
