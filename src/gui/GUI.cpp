#include <cstdio>
#include <cmath>
#include <chrono>

#include "gui/GUI.hpp"
#include "gui/GuiParameterRenderer.hpp"
#include "core/SimState.hpp"
#include "core/Types.hpp"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_stdlib.h"


// ============================================================
// Static
// ============================================================

GUI* GUI::instance = nullptr;

// ============================================================
// Platform init
// ============================================================

bool GUI::initializePlatform(const char* title) {
    instance = this;

    glfwSetErrorCallback(glfwErrorCallback);

    if (!glfwInit())
        return false;

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    window = glfwCreateWindow(
        mode->width,
        mode->height,
        title,
        nullptr,
        nullptr
    );

    if (!window) {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(window);
        glfwTerminate();
        return false;
    }

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    interaction.active = false;
    interaction.worldX = -1.0f;
    interaction.worldY = -1.0f;
    interaction.lastWorldX = -1.0f;
    interaction.lastWorldY = -1.0f;
    interaction.timestamp = std::chrono::high_resolution_clock::now();
    interaction.lastTimestamp = interaction.timestamp;

    return true;
}

// ============================================================
// ImGui init
// ============================================================

void GUI::initializeImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

// ============================================================
// Frame control
// ============================================================

bool GUI::isRunning() const {
    return window && !glfwWindowShouldClose(window);
}

void GUI::beginFrame(float worldW, float worldH) {
    glfwPollEvents();

    // Continuous interaction while mouse is pressed
    interaction.active = false;
    if (mousePressed) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);

        float wx, wy;
        if (screenToWorld((float)mx, (float)my, wx, wy)) {
            MouseInteractionType type = (activeMouseButton == GLFW_MOUSE_BUTTON_LEFT) ? 
                MouseInteractionType::LeftClickOnWorld : MouseInteractionType::RightClickOnWorld;
            interaction.type = type;
            interaction.active = true;
            interaction.worldX = wx;
            interaction.worldY = wy;
            interaction.timestamp = std::chrono::high_resolution_clock::now();
        }
    }

    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);

    glViewport(0, 0, fbw, fbh);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    updateWorldView(fbw, fbh, worldW, worldH);
}

void GUI::render(Config& simConfig, SimState& simState) {
    renderWorld(simState);
    renderControlGui(simConfig, simState);
}

void GUI::endFrame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
}

// ============================================================
// Shutdown
// ============================================================

void GUI::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }

    glfwTerminate();
    instance = nullptr;
}

// ============================================================
// GUI panels
// ============================================================

void GUI::renderControlGui(Config& simConfig, SimState& simState) {
    const bool isOpened = ImGui::Begin("Simulation");

    // Do not submit items when closed
    if (!isOpened) {
        ImGui::End();
        return;
    }

    renderParameter(simState.version);

    ImGui::Separator();
    ImGui::TextUnformatted("Simulation State");

    renderParameter(simState.paused);
    renderParameter(simState.dimensions);
    renderParameter(simState.basicBoidCountTarget);
    renderParameter(simState.predatorBoidCountTarget);
    renderParameter(simState.leftMouseEffect);
    renderParameter(simState.rightMouseEffect);
    renderParameter(simState.resetVersionSettings);
    renderParameter(simState.resetSimulation);
    renderParameter(simState.deleteObstacles);

    ImGui::Separator();
    ImGui::TextUnformatted("Version Parameters");

    // Render parameters grouped by their group names
    for (const std::string& group : simConfig.getGroupOrder()) {

        ImGui::Separator();
        ImGui::TextUnformatted(group.c_str());

        const auto& paramsInGroup = simConfig.getGroupParams().at(group);

        // render parameters in this group (name → ConfigParameter lookup)
        for (const std::string& paramName : paramsInGroup) {
            auto& p = simConfig.get(paramName);

            ImGui::PushID(&p);
            renderParameter(p);
            ImGui::PopID();
        }
    }

    ImGui::Separator();
    ImGui::Text("Interaction: %u", interaction.active ? 1 : 0);

    ImGui::End();
}

// ============================================================
// World rendering
// ============================================================

ImVec2 GUI::worldToScreen(const Vec3& p) const {
    return {
        worldView.originX + p.x * worldView.pixelsPerWorldUnit,
        worldView.originY + p.y * worldView.pixelsPerWorldUnit
    };
}

static inline float depthScale(float z, float worldZ) {
    if (z < 0.0f || z > worldZ) return 0.0f;
    return 1.0f - (z / worldZ);
}

void GUI::renderWorld(SimState& simState) {
    ImDrawList* draw = ImGui::GetBackgroundDrawList();

    const float worldW = simState.worldX.number();
    const float worldH = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    draw->AddRect(
        { worldView.originX, worldView.originY },
        { worldView.originX + worldView.viewWpx,
          worldView.originY + worldView.viewHpx },
        IM_COL32(120, 120, 120, 255)
    );

    auto drawBoidByIndex = [&](size_t idx) {
        if (idx >= simState.boids.size())
            return;

        const Boid& b = simState.boids[idx];

        if (b.pos.x < 0 || b.pos.x > worldW) return;
        if (b.pos.y < 0 || b.pos.y > worldH) return;
        if (b.pos.z < 0 || b.pos.z > worldZ) return;

        const float z = depthScale(b.pos.z, worldZ);
        if (z <= 0.0f) return;

        const ImVec2 p = worldToScreen(b.pos);
        const float r =
            b.radius * worldView.pixelsPerWorldUnit * (0.3f + 0.7f * z);

        Color color = parseColorString(b.color);

        draw->AddCircleFilled(
            p,
            r,
            IM_COL32(
                int(color.r),
                int(color.g),
                int(color.b),
                int(color.a)
            ),
            12
        );
    };

    for (size_t idx : simState.basicBoidIndices)
        drawBoidByIndex(idx);

    for (size_t idx : simState.predatorBoidIndices)
        drawBoidByIndex(idx);

    for (size_t idx : simState.obstacleBoidIndices)
        drawBoidByIndex(idx);
}

// ============================================================
// World view helpers
// ============================================================

void GUI::updateWorldView(int fbw, int fbh, float worldW, float worldH) {
    const float pad = 24.0f;

    const float availW = fbw - 2.0f * pad;
    const float availH = fbh - 2.0f * pad;

    const float scale = std::min(availW / worldW, availH / worldH);

    worldView.pixelsPerWorldUnit = scale;
    worldView.viewWpx = worldW * scale;
    worldView.viewHpx = worldH * scale;

    worldView.originX = (fbw - worldView.viewWpx) * 0.5f;
    worldView.originY = (fbh - worldView.viewHpx) * 0.5f;
}

bool GUI::screenToWorld(float sx, float sy, float& wx, float& wy) const {
    if (sx < worldView.originX || sy < worldView.originY) return false;
    if (sx >= worldView.originX + worldView.viewWpx) return false;
    if (sy >= worldView.originY + worldView.viewHpx) return false;

    wx = (sx - worldView.originX) / worldView.pixelsPerWorldUnit;
    wy = (sy - worldView.originY) / worldView.pixelsPerWorldUnit;
    return true;
}

// ============================================================
// Callbacks
// ============================================================

void GUI::glfwErrorCallback(int e, const char* d) {
    std::fprintf(stderr, "GLFW error %d: %s\n", e, d);
}

void GUI::keyCallback(GLFWwindow* w, int key, int, int action, int) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(w, GLFW_TRUE);
}

void GUI::mouseButtonCallback(GLFWwindow*, int button, int action, int) {
    if (!instance)
        return;

    if (button != GLFW_MOUSE_BUTTON_LEFT &&
        button != GLFW_MOUSE_BUTTON_RIGHT)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    if (action == GLFW_PRESS) {
        instance->mousePressed = true;
        instance->activeMouseButton = button;
    }

    if (action == GLFW_RELEASE) {
        instance->mousePressed = false;
        instance->activeMouseButton = -1;
    }
}

// ============================================================
// Accessors
// ============================================================

MouseInteractionEvent& GUI::getInteraction() {
    return interaction;
}

// ============================================================
// Color parsing (from string like BLUE, RED, GREEN, etc.)
// ============================================================
Color GUI::parseColorString(const std::string& colorStr) const {
    if (colorStr == "RED") {
        return Color{255, 0, 0, 255};
    } else if (colorStr == "GREEN") {
        return Color{0, 255, 0, 255};
    } else if (colorStr == "BLUE") {
        return Color{0, 0, 255, 255};
    } else if (colorStr == "YELLOW") {
        return Color{255, 255, 0, 255};
    } else if (colorStr == "CYAN") {
        return Color{0, 255, 255, 255};
    } else if (colorStr == "MAGENTA") {
        return Color{255, 0, 255, 255};
    } else if (colorStr == "WHITE") {
        return Color{255, 255, 255, 255};
    } else if (colorStr == "BLACK") {
        return Color{0, 0, 0, 255};
    } else if (colorStr == "GRAY") {
        return Color{128, 128, 128, 255};
    } else {
        // Default to white if unknown
        return Color{255, 255, 255, 255};
    }
}
