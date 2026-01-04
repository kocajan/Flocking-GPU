#include <cmath>
#include <cstdio>
#include <filesystem>

#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "lab/RenderFramesExperiment.hpp"

// =============================
// Image libs
// =============================

// PNG writer
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// PNG reader (for GIF composition)
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// gif-h encoder
#include "gif.h"


// ============================================================
// Depth scale — same semantics as GUI
// ============================================================

static inline float depthScale(float z, float worldZ)
{
    if (z < 0.0f || z > worldZ) return 0.0f;
    return 1.0f - (z / worldZ);
}


// ============================================================
// Color parsing — copied from GUI::parseColorString
// ============================================================

static Color parseColorString(const std::string& colorStr)
{
    if (colorStr == "RED")      return {255, 0, 0, 255};
    if (colorStr == "GREEN")    return {0, 255, 0, 255};
    if (colorStr == "BLUE")     return {0, 0, 255, 255};
    if (colorStr == "YELLOW")   return {255, 255, 0, 255};
    if (colorStr == "CYAN")     return {0, 255, 255, 255};
    if (colorStr == "MAGENTA")  return {255, 0, 255, 255};
    if (colorStr == "WHITE")    return {255, 255, 255, 255};
    if (colorStr == "BLACK")    return {0, 0, 0, 255};
    if (colorStr == "GRAY")     return {128, 128, 128, 255};
    return {255, 255, 255, 255};
}


// ============================================================
// Framebuffer helpers
// ============================================================

void RenderFramesExperiment::clearFrame(uint8_t r, uint8_t g, uint8_t b)
{
    framebuffer.assign(imgW * imgH * 4, 255);

    for (int i = 0; i < imgW * imgH; ++i) {
        framebuffer[i*4 + 0] = r;
        framebuffer[i*4 + 1] = g;
        framebuffer[i*4 + 2] = b;
    }
}

void RenderFramesExperiment::drawCircle(
    float cx, float cy, float rad,
    uint8_t R, uint8_t G, uint8_t B, uint8_t A)
{
    int x0 = std::max(0,      (int)std::floor(cx - rad));
    int x1 = std::min(imgW-1, (int)std::ceil (cx + rad));
    int y0 = std::max(0,      (int)std::floor(cy - rad));
    int y1 = std::min(imgH-1, (int)std::ceil (cy + rad));

    float r2 = rad * rad;

    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
        {
            float dx = x - cx;
            float dy = y - cy;
            if (dx*dx + dy*dy > r2)
                continue;

            int idx = (y * imgW + x) * 4;
            framebuffer[idx+0] = R;
            framebuffer[idx+1] = G;
            framebuffer[idx+2] = B;
            framebuffer[idx+3] = A;
        }
}


// ============================================================
// World → screen mapping (same math as GUI::updateWorldView)
// ============================================================

static RenderFramesExperiment::WorldView makeWorldView(int imgW, int imgH, float worldW, float worldH) {
    const float pad = 24.0f;

    float availW = imgW - 2.0f * pad;
    float availH = imgH - 2.0f * pad;

    float scale = std::min(availW / worldW, availH / worldH);

    RenderFramesExperiment::WorldView vw;
    vw.pixelsPerWorldUnit = scale;
    vw.viewWpx = worldW * scale;
    vw.viewHpx = worldH * scale;

    vw.originX = (imgW - vw.viewWpx) * 0.5f;
    vw.originY = (imgH - vw.viewHpx) * 0.5f;

    return vw;
}

RenderFramesExperiment::Vec2 RenderFramesExperiment::worldToScreen(const Vec3& p) {
    return {
        worldView.originX + p.x * worldView.pixelsPerWorldUnit,
        worldView.originY + p.y * worldView.pixelsPerWorldUnit
    };
}


// ============================================================
// Boid rendering (mirrors GUI renderWorld semantics)
// ============================================================

void RenderFramesExperiment::renderBoids()
{
    const auto& boids = simState.boids;

    const float worldZ = simState.worldZ.number();

    auto drawBoid =
        [&](const Vec3& p3, float radiusWorld, const std::string& color)
    {
        if (p3.x < 0 || p3.x > simState.worldX.number()) return;
        if (p3.y < 0 || p3.y > simState.worldY.number()) return;
        if (p3.z < 0 || p3.z > simState.worldZ.number()) return;

        float zScale = depthScale(p3.z, worldZ);
        if (zScale <= 0.0f)
            return;

        auto p2 = worldToScreen(p3);

        float r =
            radiusWorld *
            worldView.pixelsPerWorldUnit *
            (0.3f + 0.7f * zScale);

        Color c = parseColorString(color);

        drawCircle(p2.x, p2.y, r, c.r, c.g, c.b, c.a);
    };

    for (int i = 0; i < boids.basicBoidCount; ++i)
        drawBoid(boids.posBasic[i],
                 simState.basicBoidRadius.number(),
                 simState.basicBoidColor.string());

    for (int i = 0; i < boids.predatorBoidCount; ++i)
        drawBoid(boids.posPredator[i],
                 simState.predatorBoidRadius.number(),
                 simState.predatorBoidColor.string());

    for (int i = 0; i < boids.obstacleBoidCount; ++i)
        drawBoid(boids.posObstacle[i],
                 simState.obstacleBoidRadius.number(),
                 simState.obstacleBoidColor.string());
}


// ============================================================
// PNG frame output
// ============================================================

void RenderFramesExperiment::savePngFrame() {
    char name[256];
    std::snprintf(name, sizeof(name),
                  "%s/frame_%05d.png",
                  frameDir.c_str(),
                  frameIndex++);

    stbi_write_png(name,
                   imgW, imgH,
                   4,
                   framebuffer.data(),
                   imgW * 4);
}


// ============================================================
// GIF composition
// ============================================================

void RenderFramesExperiment::writeGif() {
    std::string gifPath = frameDir + "/animation.gif";

    GifWriter writer{};
    GifBegin(&writer, gifPath.c_str(), imgW, imgH, 5);

    for (int i = 0; i < frameIndex; ++i) {
        char name[256];
        std::snprintf(name, sizeof(name),
                      "%s/frame_%05d.png",
                      frameDir.c_str(), i);

        int w,h,n;
        unsigned char* img =
            stbi_load(name, &w, &h, &n, 4);

        if (!img)
            continue;

        GifWriteFrame(&writer, img, w, h, 5);
        stbi_image_free(img);
    }

    GifEnd(&writer);
}


// ============================================================
// Experiment lifecycle
// ============================================================

void RenderFramesExperiment::onVersionStart(const std::string& version) {
    this->version = version;
}

void RenderFramesExperiment::onBoidConfigStart(int boidCount) {
    frameIndex = 0;
    frameDir = experimentConfig.string("outputDirPath") + "/frames/" + version + "/boid_config_" + std::to_string(boidCount);;
    std::filesystem::create_directories(frameDir);

    framebuffer.resize(imgW * imgH * 4);

    // Match GUI camera framing
    worldView = makeWorldView(
        imgW, imgH,
        simState.worldX.number(),
        simState.worldY.number()
    );
}

void RenderFramesExperiment::onTick(int, double) {
    clearFrame(20, 20, 20);
    renderBoids();
    savePngFrame();
}

void RenderFramesExperiment::onBoidConfigEnd(int boidCount) {
    writeGif();
}
