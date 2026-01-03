#include <iostream>
#include <stdexcept>

#include "app/Application.hpp"

#include "config/ConfigLoader.hpp"
#include "config/VersionConfigLoader.hpp"
#include "simulator/SimulationUpdate.hpp"


Application::Application(const std::string& configDirPath)
    : versionManager(loadConfigs(configDirPath + "/versions")),
      simState(loadConfig(configDirPath + "/initialSimulationState.json"), versionManager.versions) {

    // Initialize platform / window
    if (!gui.initializePlatform("Flocking")) {
        std::cerr << "Failed to initialize GUI platform\n";
        initialized = false;
        return;
    }

    // Load initial simulation config for the current version
    currentVersion = simState.version.string();
    simConfig = versionManager.getSimConfig(currentVersion);

    // GUI setup
    gui.initializeImGui();

    // Mark as initialized
    initialized = true;
}

bool Application::isInitialized() {
    return initialized;
}

void Application::run() {
    if (!initialized)
        return;

    while (gui.isRunning()) {
        gui.beginFrame(simState.worldX.number(), simState.worldY.number());

        // Detect version change
        if (simState.version.string() != currentVersion) {
            currentVersion = simState.version.string();
            simConfig = versionManager.getSimConfig(currentVersion);
        }

        simulationUpdate(simState, simConfig, gui.getInteraction());

        if (simState.resetVersionSettings.binary())
            simConfig.resetAll();

        if (simState.resetSimulation.binary()) {
            simState.resetToDefaults();
            simConfig.resetAll();
        }

        gui.render(simConfig, simState);
        gui.endFrame();
    }

    gui.shutdown();
}
