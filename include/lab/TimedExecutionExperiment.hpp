#pragma once

#include "Experiment.hpp"
#include <vector>
#include <string>


struct TimedExecutionEntry {
    std::string version;
    std::vector<int> boidCounts;
    std::vector<double> avgStepTimesMs;
};

class TimedExecutionExperiment : public Experiment {
public:
    using Experiment::Experiment;

    std::vector<TimedExecutionEntry> results;

private:
    TimedExecutionEntry* currentEntry = nullptr;

    int currentSamples = 0;
    double accumulatedMs = 0.0;

public:
    void onVersionStart(const std::string& version) override;
    void onBoidConfigStart(int boidCount) override;
    void onTick(int tick, double stepMs) override;
    void onBoidConfigEnd(int boidCount) override;
    void onExperimentEnd() override;
};
