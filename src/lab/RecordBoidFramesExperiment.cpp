/**
 * \file RecordBoidFramesExperiment.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of frame recording experiment hooks.
 */

#include <cstdio>
#include <filesystem>
#include <fstream>

#include "core/SimState.hpp"
#include "core/Types.hpp"
#include "lab/RecordBoidFramesExperiment.hpp"

// -----------------------------------------------------------------------------
//  Boid count lifecycle
// -----------------------------------------------------------------------------

/**
 * \brief Called when experiment switches to a new boid count.
 *
 * Stores the active boid count to decide whether frames
 * for this configuration should be recorded.
 *
 * \param[in] boidCount Number of boids used in this run.
 */
void RecordBoidFramesExperiment::onBoidNumChangeStart(int boidCount) {
    this->boidCount = boidCount;
}

// -----------------------------------------------------------------------------
//  Version lifecycle
// -----------------------------------------------------------------------------

/**
 * \brief Called when a version run starts.
 *
 * Resets frame indices and determines whether recording is enabled
 * for this boid count. Recording occurs only for the minimum and maximum
 * boid counts specified in the experiment configuration.
 *
 * \param[in] version Identifier of the version being executed.
 */
void RecordBoidFramesExperiment::onVersionStart(const std::string& version) {
    frameIndex = 0;

    NumberRange boidRange = experimentConfig.get("numBoids").numberRange();
    record =
        (boidCount == static_cast<int>(boidRange.min)) ||
        (boidCount == static_cast<int>(boidRange.max));

    if (record) {
        frameDir =
            experimentConfig.string("outputDirPath") +
            "/boid_data/" +
            version +
            "/boid_config_" +
            std::to_string(boidCount);

        std::filesystem::create_directories(frameDir);
    }
}

// -----------------------------------------------------------------------------
//  Tick callback — write frame snapshot
// -----------------------------------------------------------------------------

/**
 * \brief Called each simulation tick to optionally write a frame file.
 *
 * Writes:
 * - world dimensions
 * - tick index
 * - one line per boid including type, position, velocity, and radius
 *
 * \param[in] tick Simulation tick index.
 * \param[in] stepMs Step duration in milliseconds (unused).
 */
void RecordBoidFramesExperiment::onTick(int tick, double) {
    if (!record)
        return;

    const auto& boids = simState.boids;

    char path[256];
    std::snprintf(
        path,
        sizeof(path),
        "%s/frame_%05d.txt",
        frameDir.c_str(),
        frameIndex++
    );

    std::ofstream out(path);
    if (!out.is_open())
        return;

    // -------------------------------------------------------------------------
    //  World info
    // -------------------------------------------------------------------------
    out << "world "
        << simState.worldX.number() << " "
        << simState.worldY.number() << " "
        << simState.worldZ.number() << '\n';

    // -------------------------------------------------------------------------
    //  Tick index
    // -------------------------------------------------------------------------
    out << "tick " << tick << '\n';

    // -------------------------------------------------------------------------
    //  Boid writer helper
    // -------------------------------------------------------------------------
    auto writeBoid =
        [&](const char* type,
            const Vec3* pos,
            const Vec3* vel,
            int count,
            float radiusWorld)
    {
        for (int i = 0; i < count; ++i) {
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
                    << '\n';
            } else {
                out << "boid "
                    << type
                    << " x " << p.x
                    << " y " << p.y
                    << " z " << p.z
                    << " vx 0"
                    << " vy 0"
                    << " vz 0"
                    << " r " << radiusWorld
                    << '\n';
            }
        }
    };

    // -------------------------------------------------------------------------
    //  Basic boids
    // -------------------------------------------------------------------------
    writeBoid(
        "basic",
        boids.posBasic.data(),
        boids.velBasic.data(),
        boids.basicBoidCount,
        simState.basicBoidRadius.number()
    );

    // -------------------------------------------------------------------------
    //  Predator boids
    // -------------------------------------------------------------------------
    writeBoid(
        "predator",
        boids.posPredator.data(),
        boids.velPredator.data(),
        boids.predatorBoidCount,
        simState.predatorBoidRadius.number()
    );

    // -------------------------------------------------------------------------
    //  Obstacle boids
    // -------------------------------------------------------------------------
    writeBoid(
        "obstacle",
        boids.posObstacle.data(),
        nullptr,
        boids.obstacleBoidCount,
        simState.obstacleBoidRadius.number()
    );
}
