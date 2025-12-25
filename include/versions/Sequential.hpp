#pragma once

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"


void sequentialSimulationStep(
    SimState& simState,
    const SimConfig& simConfig
);
