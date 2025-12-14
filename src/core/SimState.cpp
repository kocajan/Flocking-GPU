#include "core/SimState.hpp"

#include <cassert>

// ------------------------------------------------------------
// Construction
// ------------------------------------------------------------

SimState::SimState(const SimStateConfig& config) {
    resetFromConfig(config);
}

// ------------------------------------------------------------
// Reset / unpack
// ------------------------------------------------------------

void SimState::resetFromConfig(const SimStateConfig& config) {
    // Copy parameters by name from config
    dt              = config.get("dt");
    paused          = config.get("paused");
    dimensions      = config.get("dimensions");

    boidCount       = config.get("boid_count");

    leftMouseEffect  = config.get("left_mouse_effect");
    rightMouseEffect = config.get("right_mouse_effect");

    worldX = config.get("world_x");
    worldY = config.get("world_y");
    worldZ = config.get("world_z");

    // Runtime-only fields
    currentVersion = "";
    currentVersionIndex = 0;
}
