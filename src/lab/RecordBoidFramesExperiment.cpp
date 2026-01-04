#include <cstdio>
#include <filesystem>
#include <fstream>

#include "core/SimState.hpp"
#include "core/Types.hpp"
#include "lab/RecordBoidFramesExperiment.hpp"

void RecordBoidFramesExperiment::onVersionStart(const std::string& v)
{
    version = v;
}

void RecordBoidFramesExperiment::onBoidConfigStart(int boidCount)
{
    frameIndex = 0;

    frameDir =
        experimentConfig.string("outputDirPath") +
        "/boid_data/" + version +
        "/boid_config_" + std::to_string(boidCount);

    std::filesystem::create_directories(frameDir);
}

void RecordBoidFramesExperiment::onTick(int tick, double)
{
    const auto& boids = simState.boids;
    char path[256];
    std::snprintf(
        path, sizeof(path),
        "%s/frame_%05d.txt",
        frameDir.c_str(),
        frameIndex++
    );

    std::ofstream out(path);
    if (!out.is_open())
        return;

    //
    // World info (first line)
    //
    out << "world "
        << simState.worldX.number() << " "
        << simState.worldY.number() << " "
        << simState.worldZ.number() << "\n";

    //
    // Tick index
    //
    out << "tick " << tick << "\n";

    auto writeBoid =
        [&](const char* type,
            const Vec3* pos,
            const Vec3* vel,
            int count,
            float radiusWorld)
    {
        for (int i = 0; i < count; ++i)
        {
            const Vec3& p = pos[i];

            if (vel != nullptr) {
                const Vec3& v = vel[i];
                out << "boid "
                    << type
                    << " x " << p.x
                    << " y " << p.y
                    << " z " << p.z
                    << " vx " << v.x
                    << " vy " << v.y
                    << " vz " << v.z
                    << " r " << radiusWorld
                    << "\n";
            } else {
                out << "boid "
                    << type
                    << " x " << p.x
                    << " y " << p.y
                    << " z " << p.z
                    << " vx " << 0.0f
                    << " vy " << 0.0f
                    << " vz " << 0.0f
                    << " r " << radiusWorld
                    << "\n";
            }
        }
    };

    //
    // Basic boids
    //
    writeBoid(
        "basic",
        boids.posBasic.data(),
        boids.velBasic.data(),
        boids.basicBoidCount,
        simState.basicBoidRadius.number()
    );

    //
    // Predator boids
    //
    writeBoid(
        "predator",
        boids.posPredator.data(),
        boids.velPredator.data(),
        boids.predatorBoidCount,
        simState.predatorBoidRadius.number()
    );

    //
    // Obstacle boids
    //
    writeBoid(
        "obstacle",
        boids.posObstacle.data(),
        nullptr,
        boids.obstacleBoidCount,
        simState.obstacleBoidRadius.number()
    );
}
