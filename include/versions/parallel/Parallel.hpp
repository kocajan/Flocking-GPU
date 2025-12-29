#pragma once

#include "core/SimState.hpp"
#include "config/Config.hpp"


void parallelSimulationStep(SimState& simState, const Config& simConfig);
