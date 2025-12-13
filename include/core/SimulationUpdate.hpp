#pragma once

#include "SimulationState.hpp"

// Simple evolving pattern so GUI has something to show
inline void updateSimulationDummy(SimulationState& state) {
    state.tick++;

    if (state.tick % 60 == 0) {
        int i = (state.tick / 60) % (state.width * state.height);
        int x = i % state.width;
        int y = i / state.width;

        state.grid[y][x] += 0.1f;
        if (state.grid[y][x] > 1.0f)
            state.grid[y][x] = 0.0f;
    }
}
