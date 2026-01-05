/**
 * \file RecordBoidFramesExperiment.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Experiment that records per-tick boid state snapshots to text files.
 */

#pragma once

#include <string>
#include <filesystem>
#include <fstream>

#include "Experiment.hpp"

/**
 * \class RecordBoidFramesExperiment
 * \brief Records per-tick boid snapshots for selected boid counts and versions.
 *
 * Produces structured text frame dumps that capture world state, tick index,
 * and per-boid attributes. Recording is enabled only for the minimum and
 * maximum boid counts within the configured experiment range.
 */
class RecordBoidFramesExperiment : public Experiment {
public:
    using Experiment::Experiment;

public:
    void onVersionStart(const std::string& version) override;
    void onBoidNumChangeStart(int boidCount) override;
    void onTick(int tick, double stepMs) override;

private:
    int boidCount = 0;
    bool record = true;
    std::string frameDir;
    int frameIndex = 0;
};
