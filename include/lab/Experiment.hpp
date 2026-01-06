/**
 * \file Experiment.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Base interface for experiment scenario callbacks.
 */

#pragma once

#include <string>

#include "config/Config.hpp"
#include "core/SimState.hpp"

/**
 * \class Experiment
 * \brief Base interface for experiment scenario callbacks.
 *
 * Provides lifecycle hooks called by ExperimentLab during:
 * - experiment execution
 * - boid count sweeps
 * - version iterations
 * - per-tick simulation steps
 *
 * Derived classes override whichever callbacks they need.
 */
class Experiment {
public:
    SimState& simState;
    Config& simConfig;
    const Config& experimentConfig;

public:
    Experiment(SimState& simState_,
               Config& simConfig_,
               const Config& experimentConfig_);

    virtual ~Experiment() = default;

    virtual void onExperimentStart();
    virtual void onBoidNumChangeStart(int boidCount);
    virtual void onVersionStart(const std::string& version);
    virtual void onTick(int tick, double stepMs);
    virtual void onVersionEnd(const std::string& version);
    virtual void onBoidNumChangeEnd(int boidCount);
    virtual void onExperimentEnd();
};
