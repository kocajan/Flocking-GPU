#pragma once

#include <string>

#include "boids/Boid.hpp"
#include "config/SimStateConfig.hpp"
#include "config/ConfigParameter.hpp"

class SimState {
public:
    // --------------------------------------------------------
    // Construction
    // --------------------------------------------------------
    explicit SimState(const SimStateConfig& config, ConfigParameter versionParam);

    // --------------------------------------------------------
    // Unpacked parameters (still ConfigParameter)
    // --------------------------------------------------------
    ConfigParameter dt;
    ConfigParameter paused;
    ConfigParameter device;
    ConfigParameter dimensions;

    ConfigParameter boidCount;

    ConfigParameter leftMouseEffect;
    ConfigParameter rightMouseEffect;

    ConfigParameter worldX;
    ConfigParameter worldY;
    ConfigParameter worldZ;

    // --------------------------------------------------------
    // Additional runtime-only state (later)
    // --------------------------------------------------------
    ConfigParameter version;
    int tick;
    std::vector<Boid> boids;

    // --------------------------------------------------------
    // Utilities
    // --------------------------------------------------------
    void resetFromConfig(const SimStateConfig& config, ConfigParameter versionParam);
};
