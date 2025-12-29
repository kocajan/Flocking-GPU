#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "simulator/SimulationStep.hpp"

#include "versions/parallel/Parallel.hpp"
#include "versions/sequential/Sequential.hpp"
#include "versions/parallelNaive/ParallelNaive.hpp"
#include "versions/sequentialNaive/SequentialNaive.hpp"
#include "versions/parallel/ParallelSimConfigUpdate.hpp"
#include "versions/sequential/SequentialSimConfigUpdate.hpp"
#include "versions/parallelNaive/ParallelNaiveSimConfigUpdate.hpp"
#include "versions/sequentialNaive/SequentialNaiveParameters.hpp"


void simulationStep(SimState& simState, Config& simConfig)
{
    const std::string& version = simState.version.string();

    if (version == "sequentialNaive") {
        SequentialNaiveParameters parameters(simState, simConfig);
        simulationStepSequentialNaive(parameters);
    }
    else if (version == "sequential") {
        sequentialSimConfigUpdate(simConfig);
        sequentialSimulationStep(simState, simConfig);
    }
    else if (version == "parallelNaive") {
        parallelNaiveSimConfigUpdate(simConfig);
        parallelNaiveSimulationStep(simState, simConfig);
    }
    else if (version == "parallel") {
        parallelSimConfigUpdate(simConfig);
        parallelSimulationStep(simState, simConfig);
    }
    else {
        throw std::runtime_error("Unknown simulation version: " + version);
    }
}
