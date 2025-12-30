#include <cmath>
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

    basicBoidCountTarget = config.get("basicBoidCount");
    predatorBoidCountTarget = config.get("predatorBoidCount");

    leftMouseEffect = config.get("leftMouseEffect");
    rightMouseEffect = config.get("rightMouseEffect");

    resetVersionSettings = config.get("resetVersionSettings");
    resetSimulation = config.get("resetSimulation");
    deleteObstacles = config.get("deleteObstacles");
    resetVersionSettings.render = ParamRender::Button;
    resetSimulation.render = ParamRender::Button;
    deleteObstacles.render = ParamRender::Button;

    worldX = config.get("worldX");
    worldY = config.get("worldY");
    worldZ = config.get("worldZ");

    basicBoidColor = config.get("basicBoidColor");
    predatorBoidColor = config.get("predatorBoidColor");
    obstacleBoidColor = config.get("obstacleColor");

    obstacleRadius = config.get("obstacleRadius");
    predatorRadius = config.get("predatorBoidRadius");
    basicBoidRadius = config.get("basicBoidRadius");

    maxBoidPopulationChangeRate = config.get("maxBoidPopulationChangeRate");

    initialAxialSpeedRange = config.get("initialAxialSpeedRange");
    
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
