#pragma once

#include <vector>

#include "core/SimState.hpp"
#include "config/Config.hpp"
#include "gui/GUI.hpp"


void simulationUpdate(SimState& simState, const Config& simConfig, const MouseInteractionEvent& interaction);
