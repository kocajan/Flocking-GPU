/**
 * \file TimedExecutionExperiment.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Experiment for aggregating and exporting simulation timing statistics.
 */

#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include "Experiment.hpp"

/**
 * \struct TimedExecutionEntry
 * \brief Aggregated timing statistics for a single simulation version.
 *
 * Stores:
 * - version identifier
 * - list of boid counts evaluated
 * - average simulation step duration for each boid count
 */
struct TimedExecutionEntry {
    std::string version;
    std::vector<int> boidCounts;
    std::vector<double> avgStepTimesMs;
};

/**
 * \class TimedExecutionExperiment
 * \brief Measures per-version average simulation tick duration.
 *
 * Aggregates timing samples for:
 * - each simulation version
 * - each evaluated boid count
 *
 * Results are exported to CSV files at experiment completion.
 */
class TimedExecutionExperiment : public Experiment {
public:
    using Experiment::Experiment;

public:
    std::unordered_map<std::string, TimedExecutionEntry*> results;

private:
    TimedExecutionEntry* currentEntry = nullptr;

    int currentSamples = 0;
    double accumulatedMs = 0.0;

public:
    void onExperimentStart() override;
    void onVersionStart(const std::string& version) override;
    void onBoidNumChangeStart(int boidCount) override;
    void onTick(int tick, double stepMs) override;
    void onVersionEnd(const std::string& version) override;
    void onExperimentEnd() override;
};
