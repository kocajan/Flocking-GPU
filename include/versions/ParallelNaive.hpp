#pragma once

#include "core/SimState.hpp"
#include "config/Config.hpp"


void parallelNaiveSimulationStep(SimState& simState, const Config& simConfig);