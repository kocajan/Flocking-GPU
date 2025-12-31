#include <cmath>
#include <cassert>

#include "core/SimState.hpp"


//------------------------------------------------------------------------------
// Constructor
// Reset to given configuration.
//------------------------------------------------------------------------------
SimState::SimState(const Config& config, ConfigParameter versionParam) {
    resetToNewConfig(config, versionParam);
}


//------------------------------------------------------------------------------
// Reset from new configuration (runtime reconfiguration)
//------------------------------------------------------------------------------
void SimState::resetToNewConfig(const Config& config, ConfigParameter versionParam) {
    // Store baseline so this configuration can be restored later
    initialConfig = config;
    initialVersionParam = versionParam;

    //-------------------------------------------------------------------------
    // Simulation control
    //-------------------------------------------------------------------------
    dt         = config.get("dt");
    paused     = config.get("paused");
    dimensions = config.get("dimensions");

    tick = 0;

    //-------------------------------------------------------------------------
    // Population targets
    //-------------------------------------------------------------------------
    basicBoidCountTarget   = config.get("basicBoidCount");
    predatorBoidCountTarget = config.get("predatorBoidCount");

    //-------------------------------------------------------------------------
    // Interaction / input behavior
    //-------------------------------------------------------------------------
    leftMouseEffect  = config.get("leftMouseEffect");
    rightMouseEffect = config.get("rightMouseEffect");

    //-------------------------------------------------------------------------
    // Reset / control actions (UI buttons)
    //-------------------------------------------------------------------------
    resetVersionSettings = config.get("resetVersionSettings");
    resetSimulation      = config.get("resetSimulation");
    deleteObstacles      = config.get("deleteObstacles");

    resetVersionSettings.render = ParamRender::Button;
    resetSimulation.render      = ParamRender::Button;
    deleteObstacles.render      = ParamRender::Button;

    //-------------------------------------------------------------------------
    // World geometry
    //-------------------------------------------------------------------------
    worldX = config.get("worldX");
    worldY = config.get("worldY");
    worldZ = config.get("worldZ");

    //-------------------------------------------------------------------------
    // Rendering colors
    //-------------------------------------------------------------------------
    basicBoidColor    = config.get("basicBoidColor");
    predatorBoidColor = config.get("predatorBoidColor");
    obstacleBoidColor = config.get("obstacleColor");

    //-------------------------------------------------------------------------
    // Radii / collision footprint
    //-------------------------------------------------------------------------
    obstacleRadius = config.get("obstacleRadius");
    predatorRadius = config.get("predatorBoidRadius");
    basicBoidRadius = config.get("basicBoidRadius");

    //-------------------------------------------------------------------------
    // Population dynamics / constraints
    //-------------------------------------------------------------------------
    maxBoidPopulationChangeRate = config.get("maxBoidPopulationChangeRate");

    // Initial random velocity range
    initialAxialSpeedRange = config.get("initialAxialSpeedRange");

    // Numerical tolerance
    eps = config.get("eps");

    //-------------------------------------------------------------------------
    // Version selector
    //-------------------------------------------------------------------------
    version = versionParam;

    //-------------------------------------------------------------------------
    // Reset boid population state
    //-------------------------------------------------------------------------
    boids.clear();        // clears arrays, indices, counters, free list
}


//------------------------------------------------------------------------------
// Reset to original startup configuration
//------------------------------------------------------------------------------
void SimState::resetToDefaults() {
    resetToNewConfig(initialConfig, initialVersionParam);
}
