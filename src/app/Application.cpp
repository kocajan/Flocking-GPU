#include <iostream>

#include "app/Application.hpp"

#include "gui/GUI.hpp"
#include "core/SimulationState.hpp"
#include "core/SimulationUpdate.hpp"
#include "config/SimConfig.hpp"
#include "config/ConfigParameter.hpp"
#include "core/VersionManager.hpp"
#include "config/VersionConfigLoader.hpp"
#include "config/SimStateConfigLoader.hpp"
#include "config/SimStateConfig.hpp"
#include "core/SimState.hpp"


void Application::run() {
    GUI gui;

    if (!gui.initializePlatform("Flocking")) {
        return;
    }

    // Load default simulation state configuration
    const SimStateConfig simStateConfig = loadSimStateConfig("cfg/initial_simulation_state.json");
    SimState simState(simStateConfig);

    // Load version configurations
    const std::vector<VersionConfig> versionConfigs = loadVersionConfigs("cfg/versions.json");
    VersionManager versionManager(versionConfigs);

    // Get available versions
    const std::vector<std::string> availableVersions = versionManager.getAvailableVersions();

    // If no versions, exit
    if (availableVersions.empty()) {
        std::cerr << "No versions available!" << std::endl;
        return;
    }

    // Select version
    int currentVersionIndex = 0;
    std::string currentVersion = availableVersions[currentVersionIndex];

    // Save it to the simState
    simState.currentVersion = currentVersion;
    simState.currentVersionIndex = currentVersionIndex;

    gui.initializeImGui(versionManager.getSimConfig(currentVersion), simState);

    SimulationState state;
    while (gui.isRunning()) {
        gui.clearInteractions();
        gui.beginFrame();
        
        const SimConfig& cfg = gui.getSimConfig();

        if (!gui.isPaused()) {
            updateSimulationDummy(state);
        }

        gui.render(state);
        gui.endFrame();

        // If version changed, update sim config
        if (currentVersionIndex != gui.getCurrentVersionIdx()) {
            currentVersionIndex = gui.getCurrentVersionIdx();
            currentVersion = availableVersions[currentVersionIndex];
            gui.setSimConfig(versionManager.getSimConfig(currentVersion));
        }

        // Print the parameters (they might be custom based on the version) Put them on one line so I can see changes easily
        std::cout << "Version: " << currentVersion << " | ";
        std::cout << "IsPaused: " << (gui.isPaused() ? "true" : "false") << " | ";
        for (const auto& param : cfg.getParameters()) {
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
