#pragma once

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"


void sequentialNaiveSimulationStep(
    SimState& simState,
    const SimConfig& simConfig
);
