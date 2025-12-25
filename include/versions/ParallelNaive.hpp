#pragma once

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"


void parallelNaiveSimulationStep(
    SimState& simState,
    const SimConfig& simConfig
);
