#include "app/Application.hpp"

#include "gui/GUI.hpp"
#include "core/SimulationState.hpp"
#include "core/SimulationUpdate.hpp"
#include "core/SimConfig.hpp"
#include "core/ConfigParameter.hpp"

// ------------------------------------------------------------

static SimConfig makeDefaultSimConfig() {
    SimConfig cfg;

    cfg.add(ConfigParameter::Binary(
        "paused", "Paused", "Pause simulation", false
    ));

    cfg.add(ConfigParameter::Number(
        "time_scale", "Time scale", "Simulation speed multiplier",
        1.0f, 0.0f, 4.0f, 0.01f
    ));

    cfg.add(ConfigParameter::Enum(
        "color_scheme", "Color Scheme", "Grid color scheme",
        "Grayscale",
        {"Grayscale", "Heatmap", "Blue-Red"}
    ));

    cfg.add(ConfigParameter::String(
        "username", "Username", "Name of the user",
        "Guest", true
    ));

    cfg.add(ConfigParameter::String(
        "mode", "Mode", "Operating mode",
        "Normal", false,
        {"Normal", "Advanced", "Expert"}
    ));

    return cfg;
}

// ------------------------------------------------------------

void Application::run() {
    GUI gui;

    if (!gui.initializePlatform("Flocking"))
        return;

    gui.initializeImGui(makeDefaultSimConfig());

    SimulationState state;

    while (gui.isRunning()) {
        gui.clearInteractions();
        gui.beginFrame();

        const SimConfig& cfg = gui.getSimConfig();
        if (!cfg.binary("paused")) {
            updateSimulationDummy(state);
        }

        gui.render(state);
        gui.endFrame();
    }

    gui.shutdown();
}
