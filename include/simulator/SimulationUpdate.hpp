#pragma once

#include <vector>

#include "gui/GUI.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


void simulationUpdate(SimState& simState, Config& simConfig, MouseInteractionEvent& interaction);
