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

static constexpr float DRAG_THRESHOLD_PX  = 6.0f;
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

void GUI::beginFrame() {
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

    updateGridRect(fbw, fbh);
}

void GUI::render(SimConfig& simConfig, SimState& simState) {
    renderGrid(simState);
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
// GUI rendering
// ============================================================
void GUI::renderControlGui(SimConfig& simConfig, SimState& simState) {
    ImGui::Begin("Simulation");
    ImGui::Separator();

    // --------------------------------------------------------
    // Version selector
    // --------------------------------------------------------
    renderParameter(simState.version);

    // --------------------------------------------------------
    // Simulation state (explicit)
    // --------------------------------------------------------
    ImGui::TextUnformatted("Simulation State");
    renderParameter(simState.paused);
    renderParameter(simState.boidCount);
    renderParameter(simState.leftMouseEffect);
    renderParameter(simState.rightMouseEffect);

    ImGui::Separator();

    // --------------------------------------------------------
    // Version-specific parameters (generic)
    // --------------------------------------------------------
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


void GUI::renderGrid(SimState& simState) {
    ImDrawList* draw = ImGui::GetBackgroundDrawList();

    int height = simState.worldY.number();
    int width  = simState.worldX.number();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int c = int(simState.grid[y][x] * 255.0f);
            ImVec2 p0(
                gridRect.originX + x * gridRect.cellSize,
                gridRect.originY + y * gridRect.cellSize
            );

            ImVec2 p1(
                p0.x + gridRect.cellSize,
                p0.y + gridRect.cellSize
            );

            draw->AddRectFilled(p0, p1, IM_COL32(c, c, c, 255));
            draw->AddRect(p0, p1, IM_COL32(40, 40, 40, 255));
        }
    }
}

// ============================================================
// Grid helpers
// ============================================================

void GUI::updateGridRect(int fbw, int fbh) {
    const float pad = 24.0f;

    const float availW = fbw - 2 * pad;
    const float availH = fbh - 2 * pad;

    const int gw = 30;
    const int gh = 20;

    float cell = std::floor(std::min(availW / gw, availH / gh));

    gridRect.cellSize = cell;
    gridRect.gridWpx  = cell * gw;
    gridRect.gridHpx  = cell * gh;
    gridRect.originX = (fbw - gridRect.gridWpx) * 0.5f;
    gridRect.originY = (fbh - gridRect.gridHpx) * 0.5f;
}

bool GUI::screenToGridWorld(float sx, float sy, float& wx, float& wy) const {
    if (sx < gridRect.originX || sy < gridRect.originY) return false;
    if (sx >= gridRect.originX + gridRect.gridWpx) return false;
    if (sy >= gridRect.originY + gridRect.gridHpx) return false;

    wx = (sx - gridRect.originX) / gridRect.cellSize;
    wy = (sy - gridRect.originY) / gridRect.cellSize;
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
    if (!instance || button != GLFW_MOUSE_BUTTON_LEFT)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    if (action == GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(glfwGetCurrentContext(), &mx, &my);

        instance->mousePressed = true;
        instance->dragActive = false;
        instance->pressX = (float)mx;
        instance->pressY = (float)my;

        float wx, wy;
        if (instance->screenToGridWorld(instance->pressX, instance->pressY, wx, wy))
            instance->interactions.push_back({ InteractionType::MouseClickOnGrid, wx, wy });
    }

    if (action == GLFW_RELEASE) {
        instance->mousePressed = false;
        instance->dragActive = false;
    }
}

void GUI::cursorPosCallback(GLFWwindow*, double mx, double my) {
    if (!instance || !instance->mousePressed)
        return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    float dx = (float)mx - instance->pressX;
    float dy = (float)my - instance->pressY;

    if (!instance->dragActive) {
        if (dx * dx + dy * dy < DRAG_THRESHOLD_PX2)
            return;
        instance->dragActive = true;
    }

    float wx, wy;
    if (instance->screenToGridWorld((float)mx, (float)my, wx, wy))
        instance->interactions.push_back({ InteractionType::MouseDragOnGrid, wx, wy });
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
