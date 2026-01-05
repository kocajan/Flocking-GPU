#include "lab/Experiment.hpp"

// Constructor
Experiment::Experiment(SimState& simState_, Config& simConfig_, const Config& experimentConfig_)
    : simState(simState_)
    , simConfig(simConfig_)
    , experimentConfig(experimentConfig_) {}

void Experiment::onExperimentStart() {}

void Experiment::onBoidNumChangeStart(int /*boidCount*/) {}

void Experiment::onVersionStart(const std::string& /*version*/) {}

void Experiment::onTick(int /*tick*/, double /*stepMs*/) {}

void Experiment::onVersionEnd(const std::string& /*version*/) {}

void Experiment::onBoidNumChangeEnd(int /*boidCount*/) {}

void Experiment::onExperimentEnd() {}
