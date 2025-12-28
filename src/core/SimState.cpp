#include <cassert>

#include "core/SimState.hpp"


SimState::SimState(const Config& config, ConfigParameter versionParam) {
    resetToNewConfig(config, versionParam);
}

void SimState::resetToNewConfig(const Config& config, ConfigParameter versionParam) {
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
    resetSimulation = config.get("reset_simulation");
    deleteObstacles = config.get("delete_obstacles");
    resetVersionSettings.render = ParamRender::Button;
    resetSimulation.render = ParamRender::Button;
    deleteObstacles.render = ParamRender::Button;

    worldX = config.get("world_x");
    worldY = config.get("world_y");
    worldZ = config.get("world_z");

    basicBoidColor = config.get("basic_boid_color");
    predatorBoidColor = config.get("predator_boid_color");
    obstacleBoidColor = config.get("obstacle_color");

    obstacleRadius = config.get("obstacle_radius");
    predatorRadius = config.get("predator_boid_radius");
    basicBoidRadius = config.get("basic_boid_radius");

    maxBoidPopulationChangeRate = config.get("max_boid_population_change_rate");

    initialAxialSpeedRange = config.get("initial_axial_speed_range");

    eps = config.get("eps");

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
