#include "core/SimState.hpp"

#include <cassert>

// ------------------------------------------------------------
// Construction
// ------------------------------------------------------------

SimState::SimState(const SimStateConfig& config, ConfigParameter versionParam) {
    resetFromConfig(config, versionParam);
}

// ------------------------------------------------------------
// Reset / unpack
// ------------------------------------------------------------

void SimState::resetFromConfig(const SimStateConfig& config, ConfigParameter versionParam) {
    // Copy parameters by name from config
    dt               = config.get("dt");
    paused           = config.get("paused");
    dimensions       = config.get("dimensions");

    boidCount        = config.get("boid_count");

    leftMouseEffect  = config.get("left_mouse_effect");
    rightMouseEffect = config.get("right_mouse_effect");

    worldX = config.get("world_x");
    worldY = config.get("world_y");
    worldZ = config.get("world_z");

    // Null pointer for now
    version = versionParam;
    tick = 0;

    // Create a grid based on dimensions
    int width = worldX.number();
    int height = worldY.number();
    grid = std::vector<std::vector<float>>(height, std::vector<float>(width, 0.0f));
}
