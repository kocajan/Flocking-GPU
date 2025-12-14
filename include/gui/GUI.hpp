#pragma once

#include <vector>
#include <cstdint>
#include <string>

#include "config/SimConfig.hpp"

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
    void initializeImGui(SimConfig initialSimConfig, const std::vector<std::string>& allVersions, int initialVersionIndex);

    bool isRunning() const;

    void beginFrame();
    void render(const SimulationState& state);
    void endFrame();

    void shutdown();

    // --------------------------------------------------------
    // Interactions
    // --------------------------------------------------------
    void clearInteractions();
    const std::vector<InteractionEvent>& getInteractions() const;

    // --------------------------------------------------------
    // Simulation config
    // --------------------------------------------------------
    const SimConfig& getSimConfig() const;
    void setSimConfig(const SimConfig& cfg);

    // Current version getters
    int getCurrentVersionIdx() const;
    const std::string& getCurrentVersion() const;

    // getting pause state
    bool isPaused() const;

private:
    // --------------------------------------------------------
    // Internal helpers
    // --------------------------------------------------------
    void renderControlGui();
    void renderGrid(const SimulationState& state);
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
    SimConfig simConfig;

    std::vector<std::string> availableVersions;
    int currentVersionIndex = 0;

    bool isPausedFlag = false;

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
