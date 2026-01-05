/**
 * \file ExperimentLab.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Experiment execution framework for batch simulation scenarios.
 *
 * Responsibilities:
 * - loads experiment configurations
 * - applies overrides to default simulation state/config
 * - initializes experiment data layouts
 * - runs experiment pipelines across versions & boid counts
 */

#pragma once

#include <string>
#include <vector>
#include <filesystem>

#include "config/Config.hpp"
#include "core/SimState.hpp"
#include "core/VersionManager.hpp"
#include "lab/Experiment.hpp"

/**
 * \class ExperimentLab
 * \brief Manages experiment setup, execution, and scenario iteration.
 */
class ExperimentLab {
public:

    /**
     * \brief Construct experiment lab using default configuration directory.
     *
     * Loads:
     * - version configurations
     * - default simulation state for the initial version
     *
     * \param[in] configDirPath Path to configuration directory.
     */
    ExperimentLab(const std::string& configDirPath);

    /**
     * \brief Check if initialization completed successfully.
     *
     * \return true if initialized, false otherwise.
     */
    bool isInitialized();

    /**
     * \brief Run all experiment scenarios in a directory.
     *
     * \param[in] experimentConfigsDirPath Path to experiment configuration files.
     */
    void runExperiments(const std::string& experimentConfigsDirPath);

private:
    bool initialized = false;

    VersionManager versionManager;
    SimState defaultSimState;
    Config defaultSimConfig;

private:
    SimState adjustDefaultSimStateForExperiment(const Config& experimentConfig);
    Config adjustDefaultSimConfigForExperiment(const Config& experimentConfig);
    void runExperimentScenario(Experiment& experiment);
    std::vector<std::string> getVersionsToExperimentWith(const Config& experimentConfig);

    void initializeDataForExperiment(const Config& experimentConfig, SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlain(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlainWithObstacles(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlainWithObstaclesAndPredators(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentDefault(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentDefault3D(SimState& simState, int requestedBasicBoidCount);
};
