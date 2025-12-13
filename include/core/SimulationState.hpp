#pragma once

#include <vector>

// ============================================================
// Dummy simulation state
// ============================================================

struct SimulationState {
    int tick = 0;

    int width  = 30;
    int height = 20;

    // Grid values in [0, 1]
    // Accessed as grid[y][x]
    std::vector<std::vector<float>> grid;

    SimulationState()
        : grid(height, std::vector<float>(width, 0.0f)) {}
};
