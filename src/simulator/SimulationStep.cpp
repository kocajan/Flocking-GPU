#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "simulator/SimulationStep.hpp"

#include "versions/parallel/Parallel.hpp"
#include "versions/sequential/Sequential.hpp"
#include "versions/parallelNaive/ParallelNaive.hpp"
#include "versions/sequentialNaive/SequentialNaive.hpp"
#include "versions/parallel/ParallelParameters.hpp"
#include "versions/sequential/SequentialParameters.hpp"
#include "versions/parallelNaive/ParallelNaiveParameters.hpp"
#include "versions/sequentialNaive/SequentialNaiveParameters.hpp"


void simulationStep(SimState& simState, Config& simConfig)
{
    const std::string& version = simState.version.string();

    if (version == "sequentialNaive") {
        SequentialNaiveParameters parameters(simState, simConfig);
        simulationStepSequentialNaive(parameters);
    }
    else if (version == "sequential") {
        SequentialParameters parameters(simState, simConfig);
        sequentialSimulationStep(parameters);
    }
    else if (version == "parallelNaive") {
        ParallelNaiveParameters parameters(simState, simConfig);
        parallelNaiveSimulationStep(parameters);
    }
    else if (version == "parallel") {
        ParallelParameters parameters(simState, simConfig);
        parallelSimulationStep(parameters);
    }
    else {
        throw std::runtime_error("Unknown simulation version: " + version);
    }
}
