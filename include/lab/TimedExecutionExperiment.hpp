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
