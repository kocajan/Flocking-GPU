#pragma once

#include <string>

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
    std::vector<std::vector<float>> grid;

    // --------------------------------------------------------
    // Utilities
    // --------------------------------------------------------
    void resetFromConfig(const SimStateConfig& config, ConfigParameter versionParam);
};
