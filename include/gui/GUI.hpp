#pragma once

#include <vector>
#include <cstdint>
#include <string>

#include "config/SimConfig.hpp"
#include "core/SimState.hpp"

struct SimulationState;
struct GLFWwindow;

// ============================================================
// Interaction
// ============================================================

enum class InteractionType : uint8_t {
    MouseClickOnGrid,
    MouseDragOnGrid
};

struct InteractionEvent {
    InteractionType type;
    float worldX;
    float worldY;
};

// ============================================================
// GUI
// ============================================================

class GUI {
public:
    // --------------------------------------------------------
    // Platform / lifecycle
    // --------------------------------------------------------
    bool initializePlatform(const char* title);
    void initializeImGui();

    bool isRunning() const;

    void beginFrame();
    void render(SimConfig& simConfig, SimState& simState);
    void endFrame();

    void shutdown();

    // --------------------------------------------------------
    // Interactions
    // --------------------------------------------------------
    void clearInteractions();
    const std::vector<InteractionEvent>& getInteractions() const;

private:
    // --------------------------------------------------------
    // Internal helpers
    // --------------------------------------------------------
    void renderControlGui(SimConfig& simConfig, SimState& simState);
    void renderGrid(SimState& simState);
    void updateGridRect(int fbw, int fbh);

    bool screenToGridWorld(float sx, float sy, float& wx, float& wy) const;

    // --------------------------------------------------------
    // GLFW callbacks
    // --------------------------------------------------------
    static void glfwErrorCallback(int, const char*);
    static void keyCallback(GLFWwindow*, int, int, int, int);
    static void mouseButtonCallback(GLFWwindow*, int, int, int);
    static void cursorPosCallback(GLFWwindow*, double, double);

private:
    // --------------------------------------------------------
    // State
    // --------------------------------------------------------
    GLFWwindow* window = nullptr;

    struct GridScreenRect {
        float originX = 0.0f;
        float originY = 0.0f;
        float cellSize = 1.0f;
        float gridWpx = 0.0f;
        float gridHpx = 0.0f;
    };

    GridScreenRect gridRect{};
    std::vector<InteractionEvent> interactions;

    // Drag state
    bool mousePressed = false;
    bool dragActive = false;
    float pressX = 0.0f;
    float pressY = 0.0f;

    static GUI* instance;
};
