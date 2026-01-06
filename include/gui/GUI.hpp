/**
 * \file GUI.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Graphical user interface for simulation control, rendering, and interaction.
 *
 * Provides:
 * - platform & OpenGL / ImGui initialization
 * - world rendering and parameter UI panels
 * - mouse interaction mapping (screen -> world)
 * - per–frame lifecycle control
 */

#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <chrono>

#include "config/Config.hpp"
#include "core/SimState.hpp"
#include "imgui/imgui.h"
#include "core/Types.hpp"


// Forward declaration of GLFWwindow
struct GLFWwindow;

/**
 * \class GUI
 * \brief GUI subsystem responsible for rendering, event handling, and world view.
 */
class GUI {
public:

    /**
     * \brief Initialize OS window, GL context, and platform backend.
     *
     * \param[in] title Window title string.
     * \return true on success, false on failure.
     */
    bool initializePlatform(const char* title);

    /**
     * \brief Initialize ImGui context and rendering backend.
     */
    void initializeImGui();

    /**
     * \brief Check whether the GUI window is still running.
     *
     * \return true if window is active, false if close requested.
     */
    bool isRunning() const;

    /**
     * \brief Begin a new frame, poll events, and update world view.
     *
     * \param[in] worldW World width.
     * \param[in] worldH World height.
     */
    void beginFrame(float worldW, float worldH);

    /**
     * \brief Render world and control panels.
     */
    void render(Config& simConfig, SimState& simState);

    /**
     * \brief Finalize frame and present rendered output.
     */
    void endFrame();

    /**
     * \brief Shutdown GUI, destroy window, and terminate platform state.
     */
    void shutdown();

    /**
     * \brief Get current mouse interaction state.
     *
     * \return Reference to interaction data.
     */
    MouseInteractionEvent& getInteraction();

private:
    void renderControlGui(Config& simConfig, SimState& simState);
    void renderWorld(SimState& simState);

    void updateWorldView(int fbw, int fbh, float worldW, float worldH);
    bool screenToWorld(float sx, float sy, float& wx, float& wy) const;

    static void glfwErrorCallback(int, const char*);
    static void keyCallback(GLFWwindow*, int, int, int, int);
    static void mouseButtonCallback(GLFWwindow*, int, int, int);

    ImVec2 worldToScreen(const Vec3& p) const;

    Color parseColorString(const std::string& colorStr) const;

private:
    GLFWwindow* window = nullptr;

    /**
     * \struct WorldViewRect
     * \brief Screen-space viewport bounds and scaling of world units.
     */
    struct WorldViewRect {
        float originX = 0.0f;
        float originY = 0.0f;
        float pixelsPerWorldUnit = 1.0f;
        float viewWpx = 0.0f;
        float viewHpx = 0.0f;
    };

    WorldViewRect worldView{};
    MouseInteractionEvent interaction;

    bool mousePressed = false;
    int activeMouseButton = -1;

    float pressX = 0.0f;
    float pressY = 0.0f;

    static GUI* instance;
};
