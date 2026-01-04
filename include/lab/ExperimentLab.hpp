#pragma once

#include <string>
#include <vector>
#include <filesystem>

#include "config/Config.hpp"
#include "core/SimState.hpp"
#include "core/VersionManager.hpp"
#include "lab/Experiment.hpp"


class ExperimentLab {
public:
    ExperimentLab(const std::string& configDirPath);

    bool isInitialized();

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

    void initializeDataForExperiment(const Config& experimentConfig, SimState& simState, int basicBoidCount);
    void initializeDataForExperimentPlain(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlainWithObstacles(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlainWithObstaclesAndPredators(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentDefault(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentDefault3D(SimState& simState, int requestedBasicBoidCount);
};
