#pragma once

#include <string>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "config/ConfigParameter.hpp"


//------------------------------------------------------------------------------
// Simulation State
//
// Holds:
//  - configuration parameters
//  - runtime state
//  - current boid population (SoA container)
//------------------------------------------------------------------------------
class SimState {
public:
    //-------------------------------------------------------------------------
    // Persistent initial configuration (used for reset-to-defaults)
    //-------------------------------------------------------------------------
    Config initialConfig;
    ConfigParameter initialVersionParam;

    //-------------------------------------------------------------------------
    // Simulation control
    //-------------------------------------------------------------------------
    ConfigParameter dt;              // timestep
    ConfigParameter paused;          // pause toggle
    uint64_t tick = 0;               // simulation step counter

    // Dimensions / world mode (2D / 3D)
    ConfigParameter dimensions;

    //-------------------------------------------------------------------------
    // Population targets (used by population manager)
    //-------------------------------------------------------------------------
    ConfigParameter basicBoidCountTarget;
    ConfigParameter predatorBoidCountTarget;

    //-------------------------------------------------------------------------
    // Interaction & input effects
    //-------------------------------------------------------------------------
    ConfigParameter leftMouseEffect;
    ConfigParameter rightMouseEffect;
    Interaction interaction;

    //-------------------------------------------------------------------------
    // Reset / control commands
    //-------------------------------------------------------------------------
    ConfigParameter resetVersionSettings;
    ConfigParameter resetSimulation;
    ConfigParameter deleteObstacles;

    //-------------------------------------------------------------------------
    // World geometry
    //-------------------------------------------------------------------------
    ConfigParameter worldX;
    ConfigParameter worldY;
    ConfigParameter worldZ;

    //-------------------------------------------------------------------------
    // Rendering colors
    //-------------------------------------------------------------------------
    ConfigParameter basicBoidColor;
    ConfigParameter predatorBoidColor;
    ConfigParameter obstacleBoidColor;

    //-------------------------------------------------------------------------
    // Radii / physical footprint
    //-------------------------------------------------------------------------
    ConfigParameter obstacleBoidRadius;
    ConfigParameter predatorBoidRadius;
    ConfigParameter basicBoidRadius;

    //-------------------------------------------------------------------------
    // Population dynamics / constraints
    //-------------------------------------------------------------------------
    ConfigParameter maxBoidPopulationChangeRate;

    // Initial random velocity range
    ConfigParameter initialAxialSpeedRange;

    // Numerical stability parameters
    ConfigParameter eps;

    //-------------------------------------------------------------------------
    // Version / implementation selector
    //-------------------------------------------------------------------------
    ConfigParameter version;

    //-------------------------------------------------------------------------
    // Boid population storage (SoA layout)
    //-------------------------------------------------------------------------
    Boids boids;

    //-------------------------------------------------------------------------
    // Reset helpers
    //-------------------------------------------------------------------------

    // Reset from new configuration (e.g., UI configuration change)
    void resetToNewConfig(const Config& config, ConfigParameter versionParam);

    // Reset to initial startup configuration
    void resetToDefaults();

    // Has and get parameter helpers
    bool has(const std::string& name) const;
    ConfigParameter& get(const std::string& name);
    const ConfigParameter& get(const std::string& name) const;

    // Constructor from configuration
    SimState(const Config& config, ConfigParameter versionParam);
};
