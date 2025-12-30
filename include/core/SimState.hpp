#pragma once

#include <string>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "config/ConfigParameter.hpp"


class SimState {
public:
    explicit SimState(const Config& config, ConfigParameter versionParam);

    // Initial configuration (for resets)
    Config initialConfig;
    ConfigParameter initialVersionParam;

    // Parameters from config
    ConfigParameter dt;
    ConfigParameter paused;
    ConfigParameter device;
    ConfigParameter dimensions;

    ConfigParameter basicBoidCountTarget;
    ConfigParameter predatorBoidCountTarget;

    ConfigParameter leftMouseEffect;
    ConfigParameter rightMouseEffect;

    ConfigParameter resetVersionSettings;
    ConfigParameter resetSimulation;
    ConfigParameter deleteObstacles;

    ConfigParameter worldX;
    ConfigParameter worldY;
    ConfigParameter worldZ;

    ConfigParameter basicBoidColor;
    ConfigParameter predatorBoidColor;
    ConfigParameter obstacleBoidColor;

    ConfigParameter obstacleRadius;
    ConfigParameter predatorRadius;
    ConfigParameter basicBoidRadius;

    ConfigParameter maxBoidPopulationChangeRate;

    ConfigParameter initialAxialSpeedRange;

    ConfigParameter eps;

    // Additional runtime state
    // - Version parameter containing current version and available versions
    ConfigParameter version;

    // - Simulation tick counter
    uint64_t tick;

    // - Boid population
    std::vector<Boid> boids;

    // - Boid type counters
    uint64_t basicBoidCount;
    uint64_t predatorBoidCount;
    uint64_t obstacleBoidCount;

    // - Boid type indices
    std::vector<size_t> basicBoidIndices;
    std::vector<size_t> predatorBoidIndices;
    std::vector<size_t> obstacleBoidIndices;

    // - Define interaction
    Interaction interaction;

    // - Free boid indices for reuse
    std::vector<size_t> freeBoidIndices;

    // Reset from configuration
    void resetToNewConfig(const Config& config, ConfigParameter versionParam);

    // Reset to initial configuration
    void resetToDefaults();
};
