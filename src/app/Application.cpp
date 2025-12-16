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
#include "simulator/SimulationUpdate.hpp"

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

    float worldX = simState.worldX.number();
    float worldY = simState.worldY.number();

    while (gui.isRunning()) {
        gui.clearInteractions();
        gui.beginFrame(worldX, worldY);

        simulationUpdate(simState, simConfig, gui.getInteractions());

        // Check whether version has changed
        if (simState.version.string() != currentVersion) {
            currentVersion = simState.version.string();
            simConfig = versionManager.getSimConfig(currentVersion);
        }

        // Check whether to reset to default settings
        if (simState.resetVersionSettings.binary()) {
            simState.resetToDefaults();
        }

        gui.render(simConfig, simState);
        gui.endFrame();
    }

    gui.shutdown();
}
