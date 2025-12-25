#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "simulator/SimulationStep.hpp"

#include "versions/Parallel.hpp"
#include "versions/Sequential.hpp"
#include "versions/ParallelNaive.hpp"
#include "versions/SequentialNaive.hpp"


void simulationStep(SimState& simState, const Config& simConfig)
{
    const std::string& version = simState.version.string();

    if (version == "sequentialNaive") {
        sequentialNaiveSimulationStep(simState, simConfig);
    }
    else if (version == "sequential") {
        sequentialSimulationStep(simState, simConfig);
    }
    else if (version == "parallelNaive") {
        parallelNaiveSimulationStep(simState, simConfig);
    }
    else if (version == "parallel") {
        parallelSimulationStep(simState, simConfig);
    }
    else {
        throw std::runtime_error("Unknown simulation version: " + version);
    }
}
