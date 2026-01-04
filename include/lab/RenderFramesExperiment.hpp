#pragma once

#include "Experiment.hpp"
#include <vector>
#include <string>

class RenderFramesExperiment : public Experiment {
public:
    using Experiment::Experiment;

    struct Vec2 { float x, y; };

    struct WorldView {
        float originX = 0;
        float originY = 0;
        float viewWpx = 0;
        float viewHpx = 0;
        float pixelsPerWorldUnit = 1.0f;
    };

private:
    int frameIndex = 0;
    std::string version;
    std::string frameDir;

    // Output resolution (can later be configurable)
    int imgW = 1024;
    int imgH = 768;

    // RGBA framebuffer
    std::vector<uint8_t> framebuffer;

    // Matches GUI::worldView behavior
    WorldView worldView;

public:
    void onVersionStart(const std::string& version) override;
    void onBoidConfigStart(int boidCount) override;
    void onTick(int tick, double stepMs) override;
    void onBoidConfigEnd(int boidCount) override;
    
private:
    // Rendering helpers
    void clearFrame(uint8_t r, uint8_t g, uint8_t b);
    void drawCircle(float cx, float cy, float r,
                    uint8_t R,uint8_t G,uint8_t B,uint8_t A);

    void renderBoids();
    void savePngFrame();
    void writeGif();

    Vec2 worldToScreen(const Vec3& p);
};
