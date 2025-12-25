#pragma once

#include <vector>

#include "gui/GUI.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


void simulationUpdate(SimState& simState, const Config& simConfig, const MouseInteractionEvent& interaction);
