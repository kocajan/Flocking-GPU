#pragma once

#include <string>
#include <vector>
#include <filesystem>

#include "config/Config.hpp"
#include "core/SimState.hpp"
#include "core/VersionManager.hpp"


struct ExperimentResult {
    std::string version;
    std::vector<int> boidCounts;
    std::vector<double> avgStepTimesMs;
    SimState initialSimState;
    SimState finalSimState;

    ExperimentResult(const std::string& ver, const SimState& initState)
        : version(ver), initialSimState(initState), finalSimState(initState) {}
};

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
    void runExperimentScenario(const Config& experimentConfig, SimState& simState, Config& simConfig);
    void initializeDataForExperiment(const Config& experimentConfig, SimState& simState, int basicBoidCount);
    void finalizeExperimentResults(const Config& experimentConfig, std::vector<ExperimentResult>& results);
    std::vector<std::string> getVersionsToExperimentWith(const Config& experimentConfig);

    void initializeDataForExperimentPlain(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlainWithObstacles(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentPlainWithObstaclesAndPredators(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentDefault(SimState& simState, int requestedBasicBoidCount);
    void initializeDataForExperimentDefault3D(SimState& simState, int requestedBasicBoidCount);
};
