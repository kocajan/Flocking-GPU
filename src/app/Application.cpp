#include <iostream>

#include "app/Application.hpp"

#include "gui/GUI.hpp"
#include "config/SimConfig.hpp"
#include "config/ConfigParameter.hpp"
#include "core/VersionManager.hpp"
#include "config/VersionConfigLoader.hpp"
#include "config/SimStateConfigLoader.hpp"
#include "config/SimStateConfig.hpp"
#include "core/SimState.hpp"
#include "core/SimulationUpdate.hpp"


void Application::run() {
    GUI gui;

    if (!gui.initializePlatform("Flocking")) {
        return;
    }

    const std::vector<VersionConfig> versionConfigs = loadVersionConfigs("cfg/versions.json");
    VersionManager versionManager(versionConfigs);

    const SimStateConfig simStateConfig = loadSimStateConfig("cfg/initial_simulation_state.json");
    SimState simState(simStateConfig, versionManager.versions);

    std::string currentVersion = simState.version.string();
    SimConfig simConfig = versionManager.getSimConfig(currentVersion);

    gui.initializeImGui();

    while (gui.isRunning()) {
        gui.clearInteractions();
        gui.beginFrame();

        updateSimulationDummy(simState, simConfig, gui.getInteractions());

        // Check whether version has changed
        if (simState.version.string() != currentVersion) {
            currentVersion = simState.version.string();
            simConfig = versionManager.getSimConfig(currentVersion);
        }

        gui.render(simConfig, simState);
        gui.endFrame();

        // Print the parameters (they might be custom based on the version) Put them on one line so I can see changes easily
        std::cout << "Version: " << simState.version.string() << " | ";
        std::cout << "IsPaused: " << (simState.paused.binary() ? "true" : "false") << " | ";
        std::cout << "BoidCount: " << simState.boidCount.number() << " | ";
        std::cout << "LeftMouseEffect: " << simState.leftMouseEffect.string() << " | ";
        std::cout << "RightMouseEffect: " << simState.rightMouseEffect.string() << " | ";
        for (const auto& param : simConfig.getParameters()) {
            if (param.type == ParamType::Number) {
                std::cout << param.name << "=" << param.number() << " ";
            } else if (param.type == ParamType::Binary) {
                std::cout << param.name << "=" << (param.binary() ? "true" : "false") << " ";
            } else if (param.type == ParamType::String || param.type == ParamType::Enum) {
                std::cout << param.name << "=" << param.string() << " ";
            }
        }
        std::cout << std::endl;
    }

    gui.shutdown();
}
