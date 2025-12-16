#pragma once

#include <string>

#include "boids/Boid.hpp"
#include "config/SimStateConfig.hpp"
#include "config/ConfigParameter.hpp"

class SimState {
public:
    explicit SimState(const SimStateConfig& config, ConfigParameter versionParam);

    // Initial configuration (for resets)
    SimStateConfig initialConfig;
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
    std::vector<size_t> phantomBoidIndices;

    // - Free boid indices for reuse
    std::vector<size_t> freeBoidIndices;

    // Reset from configuration
    void resetToNewConfig(const SimStateConfig& config, ConfigParameter versionParam);

    // Reset to initial configuration
    void resetToDefaults();
};
