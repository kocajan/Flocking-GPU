#pragma once

#include <vector>
#include <cstdint>

#include "config/SimConfig.hpp"
#include "core/SimState.hpp"

#include "imgui/imgui.h"

struct GLFWwindow;

// ============================================================
// Interaction
// ============================================================

enum class MouseInteractionType : uint8_t {
    LeftClickOnWorld,
    RightClickOnWorld
};

struct MouseInteractionEvent {
    MouseInteractionType type;
    bool active;
    float worldX;
    float worldY;
};

// ============================================================
// GUI
// ============================================================

class GUI {
public:
    bool initializePlatform(const char* title);
    void initializeImGui();

    bool isRunning() const;

    void beginFrame(float worldW, float worldH);
    void render(SimConfig& simConfig, SimState& simState);
    void endFrame();

    void shutdown();

    const MouseInteractionEvent& getInteraction() const;

private:
    void renderControlGui(SimConfig& simConfig, SimState& simState);
    void renderWorld(SimState& simState);

    void updateWorldView(int fbw, int fbh, float worldW, float worldH);
    bool screenToWorld(float sx, float sy, float& wx, float& wy) const;

    static void glfwErrorCallback(int, const char*);
    static void keyCallback(GLFWwindow*, int, int, int, int);
    static void mouseButtonCallback(GLFWwindow*, int, int, int);

    ImVec2 worldToScreen(const Vec3& p) const;

private:
    GLFWwindow* window = nullptr;

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
    bool dragActive = false;
    int activeMouseButton = -1;

    float pressX = 0.0f;
    float pressY = 0.0f;
    
    static GUI* instance;
};
