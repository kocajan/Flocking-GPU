#pragma once

#include <string>

#include "config/SimStateConfig.hpp"
#include "config/ConfigParameter.hpp"

class SimState {
public:
    // --------------------------------------------------------
    // Construction
    // --------------------------------------------------------
    explicit SimState(const SimStateConfig& config);

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
    std::string currentVersion = "";
    int currentVersionIndex = 0;

    // --------------------------------------------------------
    // Utilities
    // --------------------------------------------------------
    void resetFromConfig(const SimStateConfig& config);
};
