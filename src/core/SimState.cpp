#include "core/SimState.hpp"

#include <cassert>

SimState::SimState(const SimStateConfig& config, ConfigParameter versionParam) {
    resetToNewConfig(config, versionParam);
}

void SimState::resetToNewConfig(const SimStateConfig& config, ConfigParameter versionParam) {
    // Set initial config and version param
    initialConfig = config;
    initialVersionParam = versionParam;

    // Config parameters
    dt = config.get("dt");
    paused = config.get("paused");
    device = config.get("device");
    dimensions = config.get("dimensions");

    basicBoidCountTarget = config.get("basic_boid_count");
    predatorBoidCountTarget = config.get("predator_boid_count");

    leftMouseEffect = config.get("left_mouse_effect");
    rightMouseEffect = config.get("right_mouse_effect");

    resetVersionSettings = config.get("reset_version_settings");
    deleteObstacles = config.get("delete_obstacles");

    resetVersionSettings.render = ParamRender::Button;
    deleteObstacles.render = ParamRender::Button;

    worldX = config.get("world_x");
    worldY = config.get("world_y");
    worldZ = config.get("world_z");

    version = versionParam;

    tick = 0;
    basicBoidCount = 0;
    predatorBoidCount = 0;
    obstacleBoidCount = 0;

    boids.clear();
    basicBoidIndices.clear();
    predatorBoidIndices.clear();
    obstacleBoidIndices.clear();
    freeBoidIndices.clear();
}

void SimState::resetToDefaults() {
    resetToNewConfig(initialConfig, initialVersionParam);
}