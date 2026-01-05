/**
 * \file Experiment.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Default no-op implementations of experiment lifecycle hooks.
 */

#include "lab/Experiment.hpp"

// -----------------------------------------------------------------------------
//  Construction
// -----------------------------------------------------------------------------

/**
 * \brief Construct experiment wrapper around shared simulation objects.
 *
 * \param[in,out] simState_ Simulation state reference.
 * \param[in,out] simConfig_ Simulation configuration reference.
 * \param[in] experimentConfig_ Experiment configuration reference.
 */
Experiment::Experiment(SimState& simState_,
                       Config& simConfig_,
                       const Config& experimentConfig_)
    : simState(simState_)
    , simConfig(simConfig_)
    , experimentConfig(experimentConfig_) {}

// -----------------------------------------------------------------------------
//  Default lifecycle hooks (no-op)
// -----------------------------------------------------------------------------

/**
 * \brief Called before any experiment runs are executed.
 */
void Experiment::onExperimentStart() {}

/**
 * \brief Called when a new boid count sweep begins.
 *
 * \param[in] boidCount Active boid count (unused).
 */
void Experiment::onBoidNumChangeStart(int /*boidCount*/) {}

/**
 * \brief Called when a version run starts.
 *
 * \param[in] version Version identifier (unused).
 */
void Experiment::onVersionStart(const std::string& /*version*/) {}

/**
 * \brief Called on each simulation tick after warmup.
 *
 * \param[in] tick Tick index (unused).
 * \param[in] stepMs Step duration in milliseconds (unused).
 */
void Experiment::onTick(int /*tick*/, double /*stepMs*/) {}

/**
 * \brief Called when a version run finishes.
 *
 * \param[in] version Version identifier (unused).
 */
void Experiment::onVersionEnd(const std::string& /*version*/) {}

/**
 * \brief Called when a boid count sweep iteration ends.
 *
 * \param[in] boidCount Active boid count (unused).
 */
void Experiment::onBoidNumChangeEnd(int /*boidCount*/) {}

/**
 * \brief Called after all experiment runs have finished.
 */
void Experiment::onExperimentEnd() {}
