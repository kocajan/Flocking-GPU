/**
 * \file SimState.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of simulation runtime state and reset utilities.
 */

#include <cmath>
#include <cassert>

#include "core/SimState.hpp"


// Constructor — reset to given configuration.
SimState::SimState(const Config& config, ConfigParameter versionParam) {
    resetToNewConfig(config, versionParam);
}

void SimState::resetToNewConfig(const Config& config, ConfigParameter versionParam) {
    // Store baseline so this configuration can be restored later
    initialConfig = config;
    initialVersionParam = versionParam;

    // Simulation control
    dt = config.get("dt");
    paused = config.get("paused");
    dimensions = config.get("dimensions");
    tick = 0;

    // Population targets
    basicBoidCountTarget = config.get("basicBoidCount");
    predatorBoidCountTarget = config.get("predatorBoidCount");

    // Interaction / input behavior
    leftMouseEffect = config.get("leftMouseEffect");
    rightMouseEffect = config.get("rightMouseEffect");

    // Reset / control actions (UI buttons)
    resetVersionSettings = config.get("resetVersionSettings");
    resetSimulation = config.get("resetSimulation");
    deleteObstacles = config.get("deleteObstacles");

    resetVersionSettings.render = ParamRender::Button;
    resetSimulation.render = ParamRender::Button;
    deleteObstacles.render = ParamRender::Button;

    // World geometry
    worldX = config.get("worldX");
    worldY = config.get("worldY");
    worldZ = config.get("worldZ");

    // Rendering colors
    basicBoidColor = config.get("basicBoidColor");
    predatorBoidColor = config.get("predatorBoidColor");
    obstacleBoidColor = config.get("obstacleColor");

    // Radii / collision footprint
    obstacleBoidRadius = config.get("obstacleBoidRadius");
    predatorBoidRadius = config.get("predatorBoidRadius");
    basicBoidRadius = config.get("basicBoidRadius");

    // Population dynamics / constraints
    maxBoidPopulationChangeRate = config.get("maxBoidPopulationChangeRate");

    // Initial random velocity range
    initialAxialSpeedRange = config.get("initialAxialSpeedRange");

    // Numerical tolerance
    eps = config.get("eps");

    // Version selector
    version = versionParam;

    // Reset boid population state
    boids.clear();
}

void SimState::resetToDefaults() {
    resetToNewConfig(initialConfig, initialVersionParam);
}

bool SimState::has(const std::string& name) const {
    return initialConfig.has(name);
}

ConfigParameter& SimState::get(const std::string& name) {
    return initialConfig.get(name);
}

const ConfigParameter& SimState::get(const std::string& name) const {
    return initialConfig.get(name);
}
