#pragma once

#include "Experiment.hpp"
#include <string>
#include <filesystem>
#include <fstream>

class RecordBoidFramesExperiment : public Experiment {
public:
    using Experiment::Experiment;

private:
    int boidCount = 0;
    bool record = true;
    std::string frameDir;
    int frameIndex = 0;

public:
    void onVersionStart(const std::string& version) override;
    void onBoidNumChangeStart(int boidCount) override;
    void onTick(int tick, double stepMs) override;
};
