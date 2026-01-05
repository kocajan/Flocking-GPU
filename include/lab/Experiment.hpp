#pragma once

#include <string>
#include "config/Config.hpp"
#include "core/SimState.hpp"

class Experiment {
public:
    SimState& simState;
    Config& simConfig;
    const Config& experimentConfig;

    Experiment(SimState& simState_, Config& simConfig_, const Config& experimentConfig_);

    virtual ~Experiment() = default;

    virtual void onExperimentStart();
    virtual void onBoidNumChangeStart(int boidCount);
    virtual void onVersionStart(const std::string& version);
    virtual void onTick(int tick, double stepMs);
    virtual void onVersionEnd(const std::string& version);
    virtual void onBoidNumChangeEnd(int boidCount);
    virtual void onExperimentEnd();
};
