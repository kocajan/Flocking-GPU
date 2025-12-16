#include "gui/GUI.hpp"
#include "gui/GuiParameterRenderer.hpp"

#include <cstdio>
#include <cmath>

#include "glad/glad.h"
#include "GLFW/glfw3.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_stdlib.h"

#include "core/SimState.hpp"

// ============================================================
// Static
// ============================================================

GUI* GUI::instance = nullptr;

// Larger threshold to avoid jitter
static constexpr float DRAG_THRESHOLD_PX  = 12.0f;
static constexpr float DRAG_THRESHOLD_PX2 = DRAG_THRESHOLD_PX * DRAG_THRESHOLD_PX;

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
    glfwSetCursorPosCallback(window, cursorPosCallback);

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
    clearInteractions();
    glfwPollEvents();

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

void GUI::render(SimConfig& simConfig, SimState& simState) {
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

void GUI::renderControlGui(SimConfig& simConfig, SimState& simState) {
    ImGui::Begin("Simulation");

    renderParameter(simState.version);

    ImGui::Separator();
    ImGui::TextUnformatted("Simulation State");

    renderParameter(simState.paused);
    renderParameter(simState.device);
    renderParameter(simState.dimensions);
    renderParameter(simState.basicBoidCountTarget);
    renderParameter(simState.predatorBoidCountTarget);
    renderParameter(simState.leftMouseEffect);
    renderParameter(simState.rightMouseEffect);
    renderParameter(simState.resetVersionSettings);
    renderParameter(simState.deleteObstacles);

    ImGui::Separator();
    ImGui::TextUnformatted("Version Parameters");

    for (auto& p : simConfig.getParameters()) {
        ImGui::PushID(&p);
        renderParameter(p);
        ImGui::PopID();
    }

    ImGui::Separator();
    ImGui::Text("Interactions: %zu", interactions.size());

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

    // World bounds
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

        // World bounds check
        if (b.pos.x < 0 || b.pos.x > worldW) return;
        if (b.pos.y < 0 || b.pos.y > worldH) return;
        if (b.pos.z < 0 || b.pos.z > worldZ) return;

        const float z = depthScale(b.pos.z, worldZ);
        if (z <= 0.0f) return;

        const ImVec2 p = worldToScreen(b.pos);
        const float r =
            b.radius * worldView.pixelsPerWorldUnit * (0.3f + 0.7f * z);

        draw->AddCircleFilled(
            p,
            r,
            IM_COL32(
                int(b.color.r * 255.0f),
                int(b.color.g * 255.0f),
                int(b.color.b * 255.0f),
                int(b.color.a * 255.0f)
            ),
            12
        );
    };

    // Render only boids tracked by SimState
    for (size_t idx : simState.basicBoidIndices) {
        drawBoidByIndex(idx);
    }

    for (size_t idx : simState.predatorBoidIndices) {
        drawBoidByIndex(idx);
    }

    for (size_t idx : simState.obstacleBoidIndices) {
        drawBoidByIndex(idx);
    }
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
        instance->dragActive = false;

        double mx, my;
        glfwGetCursorPos(glfwGetCurrentContext(), &mx, &my);

        instance->pressX = (float)mx;
        instance->pressY = (float)my;

        float wx, wy;
        if (instance->screenToWorld(instance->pressX, instance->pressY, wx, wy)) {
            InteractionType type =
                (button == GLFW_MOUSE_BUTTON_LEFT)
                    ? InteractionType::LeftClickOnWorld
                    : InteractionType::RightClickOnWorld;

            instance->interactions.push_back({ type, wx, wy });
        }
    }

    if (action == GLFW_RELEASE) {
        instance->mousePressed = false;
        instance->activeMouseButton = -1;
        instance->dragActive = false;
    }
}

void GUI::cursorPosCallback(GLFWwindow*, double mx, double my) {
    if (!instance || !instance->mousePressed)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    const float dx = (float)mx - instance->pressX;
    const float dy = (float)my - instance->pressY;

    if (!instance->dragActive) {
        if (dx * dx + dy * dy < DRAG_THRESHOLD_PX2)
            return;
        instance->dragActive = true;
    }

    float wx, wy;
    if (!instance->screenToWorld((float)mx, (float)my, wx, wy))
        return;

    InteractionType type =
        (instance->activeMouseButton == GLFW_MOUSE_BUTTON_LEFT)
            ? InteractionType::LeftClickOnWorld
            : InteractionType::RightClickOnWorld;

    instance->interactions.push_back({ type, wx, wy });
}

// ============================================================
// Accessors
// ============================================================

void GUI::clearInteractions() {
    interactions.clear();
}

const std::vector<InteractionEvent>& GUI::getInteractions() const {
    return interactions;
}
