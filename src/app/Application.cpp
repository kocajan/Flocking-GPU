#include <iostream>
#include <filesystem>

#include "app/Application.hpp"

#include "gui/GUI.hpp"
#include "config/Config.hpp"
#include "config/ConfigParameter.hpp"
#include "core/VersionManager.hpp"
#include "config/VersionConfigLoader.hpp"
#include "config/ConfigLoader.hpp"
#include "core/SimState.hpp"
#include "simulator/SimulationUpdate.hpp"


void Application::run(const std::string& configPath) {
    GUI gui;
    if (!gui.initializePlatform("Flocking")) {
        return;
    }

    const std::filesystem::path configDir = std::filesystem::path(configPath);

    const std::vector<Config> versionConfigs = loadVersionConfigs(configDir.string() + "/versions");
    VersionManager versionManager(versionConfigs);

    const Config simStateConfig = loadConfig(configDir.string() + "/initialSimulationState.json");
    SimState simState(simStateConfig, versionManager.versions);

    std::string currentVersion = simState.version.string();
    Config simConfig = versionManager.getSimConfig(currentVersion);

    gui.initializeImGui();

    float worldX = simState.worldX.number();
    float worldY = simState.worldY.number();

    while (gui.isRunning()) {
        gui.beginFrame(worldX, worldY);

        // Check whether version has changed
        if (simState.version.string() != currentVersion) {
            currentVersion = simState.version.string();
            simConfig = versionManager.getSimConfig(currentVersion);
        }

        simulationUpdate(simState, simConfig, gui.getInteraction());

        // Check whether to reset to default settings
        if (simState.resetVersionSettings.binary()) {
            simConfig.resetAll();
        }

        // Check whether to reset simulation
        if (simState.resetSimulation.binary()) {
            simState.resetToDefaults();
            simConfig.resetAll();
        }

        gui.render(simConfig, simState);
        gui.endFrame();
    }

    gui.shutdown();
}
