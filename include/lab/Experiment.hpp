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
    virtual void onVersionStart(const std::string& version);
    virtual void onBoidConfigStart(int boidCount);
    virtual void onTick(int tick, double stepMs);
    virtual void onBoidConfigEnd(int boidCount);
    virtual void onVersionEnd(const std::string& version);
    virtual void onExperimentEnd();
};
