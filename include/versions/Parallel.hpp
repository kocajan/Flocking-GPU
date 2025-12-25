#pragma once

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"


void parallelSimulationStep(
    SimState& simState,
    const SimConfig& simConfig
);
