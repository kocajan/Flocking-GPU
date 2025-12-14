#pragma once

#include "core/SimState.hpp"
#include "config/SimConfig.hpp"
#include "gui/GUI.hpp"

// Simple evolving pattern so GUI has something to show
inline void updateSimulationDummy(SimState& simState, SimConfig& simConfig, const std::vector<InteractionEvent>& interactions) {

    if (simState.paused.binary())
        return;

    simState.tick++;

    if (simState.tick % 60 == 0) {
        int width  = static_cast<int>(simState.worldX.number());
        int height = static_cast<int>(simState.worldY.number());
        int i = (simState.tick / 60) % (width * height);
        int x = i % width;
        int y = i / width;

        simState.grid[y][x] += 0.1f;
        if (simState.grid[y][x] > 1.0f)
            simState.grid[y][x] = 0.0f;
    }
}
