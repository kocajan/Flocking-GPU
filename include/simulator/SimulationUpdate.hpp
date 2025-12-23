#pragma once

#include <vector>

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"
#include "gui/GUI.hpp"

void simulationUpdate(SimState& simState, const SimConfig& simConfig, const MouseInteractionEvent& interaction);
