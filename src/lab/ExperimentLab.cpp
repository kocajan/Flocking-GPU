#include <chrono>
#include <iostream>

#include "lab/ExperimentLab.hpp"

#include "config/ConfigLoader.hpp"
#include "config/VersionConfigLoader.hpp"
#include "simulator/SimulationStep.hpp"
#include "simulator/SimulatorHelpers.hpp"


ExperimentLab::ExperimentLab(const std::string& configDirPath)
    : versionManager(loadConfigs(configDirPath + "/versions")),
      defaultSimState(loadConfig(configDirPath + "/initialSimulationState.json"), versionManager.versions) {
    
    // Load default simulation config for the default version
    std::string currentVersion = defaultSimState.version.string();
    defaultSimConfig = versionManager.getSimConfig(currentVersion);

    // Mark as initialized
    initialized = true;
}

bool ExperimentLab::isInitialized() {
    return initialized;
}

void ExperimentLab::runExperiments(const std::string& experimentConfigsDirPath) {
    // Load experiment scenario configs
    std::vector<Config> experimentScenarioConfigs = loadConfigs(experimentConfigsDirPath);

    // Run each experiment scenario
    for (const Config& experimentConfig : experimentScenarioConfigs) {
        std::cout << "\nRunning experiment: " << experimentConfig.getConfigId() << '\n';

        // Apply experiment overrides
        SimState adjustedSimState = adjustDefaultSimStateForExperiment(experimentConfig);
        Config adjustedSimConfig = adjustDefaultSimConfigForExperiment(experimentConfig);

        // Run the experiment
        runExperimentScenario(experimentConfig, adjustedSimState, adjustedSimConfig);

        std::cout << "\nCompleted experiment: " << experimentConfig.getConfigId() << "\n\n";
    }
}

SimState ExperimentLab::adjustDefaultSimStateForExperiment(const Config& experimentConfig) {
    SimState adjustedSimState = defaultSimState;

    // Apply experiment-specific overrides to sim state parameters
    for (const auto& paramName : experimentConfig.getGroupParams().at("SimState")) {
        if (adjustedSimState.has(paramName)) {
            adjustedSimState.get(paramName).value = experimentConfig.get(paramName).value;
        }
    }

    return adjustedSimState;
}

Config ExperimentLab::adjustDefaultSimConfigForExperiment(const Config& experimentConfig) {
    Config adjustedSimConfig = defaultSimConfig;

    // Apply experiment-specific overrides to sim state parameters
    for (const auto& paramName : experimentConfig.getGroupParams().at("SimConfig")) {
        if (adjustedSimConfig.has(paramName)) {
            adjustedSimConfig.get(paramName).value = experimentConfig.get(paramName).value;
        }
    }

    return adjustedSimConfig;
}

void ExperimentLab::runExperimentScenario(const Config& experimentConfig, SimState& simState, Config& simConfig) {
    // Get version to run
    std::vector<std::string> versionsToRun = getVersionsToExperimentWith(experimentConfig);

    // Unpack experiment parameters
    const int totalTicks = static_cast<int>(experimentConfig.number("totalTicks"));
    const int warmupTicks = static_cast<int>(experimentConfig.number("warmupTicks"));

    // Get boid count range
    NumberRange numBoidsRange = experimentConfig.get("numBoids").numberRange();

    int numBoidsStep = static_cast<int>(numBoidsRange.step);
    int numBoidsMin = static_cast<int>(numBoidsRange.min);
    int numBoidsMax = static_cast<int>(numBoidsRange.max);

    std::vector<ExperimentResult> results;
    for (const std::string& version : versionsToRun) {
        printf("  - Running version: %s\n", version.c_str());
        // Set version in simulation state
        simState.version.string() = version;

        // Create result entry (intialize initial and final states)
        ExperimentResult result(version, simState);

        for (int boidCount = numBoidsMin; boidCount <= numBoidsMax; boidCount += numBoidsStep) {
            printf("    - Testing with %d boids...\n", boidCount);
            // Reset / initialize data for experiment
            initializeDataForExperiment(experimentConfig, simState, boidCount);

            // Reset simulation state
            double accumulatedStepMs = 0.0;

            for (int tick = 0; tick < totalTicks; ++tick) {
                // Measure step time
                auto t0 = std::chrono::steady_clock::now();
                simulationStep(simState, simConfig);
                auto t1 = std::chrono::steady_clock::now();

                // Skip warmup ticks
                if (tick < warmupTicks) {
                    continue;
                }

                // Save measurement
                std::chrono::duration<double, std::milli> stepMs = t1 - t0;
                accumulatedStepMs += stepMs.count();
            }

            // Compute average step time
            double avgStepMs = accumulatedStepMs / (totalTicks - warmupTicks);

            // Save result
            result.boidCounts.push_back(boidCount);
            result.avgStepTimesMs.push_back(avgStepMs);
        }
        // Save final sim state and store result
        result.finalSimState = simState;
        results.push_back(result);
    }
    // Finalize experiment results (e.g., output to console or file)
    finalizeExperimentResults(experimentConfig, results);
}

void ExperimentLab::initializeDataForExperiment(const Config& experimentConfig, SimState& simState, int basicBoidCount) {
    // Get current experiment ID
    const std::string& experimentId = experimentConfig.getConfigId();

    if (experimentId == "plainExperiment") {
        initializeDataForExperimentPlain(simState, basicBoidCount);
    } else if (experimentId == "plainWithObstaclesExperiment") {
        initializeDataForExperimentPlainWithObstacles(simState, basicBoidCount);
    } else if (experimentId == "plainWithObstaclesAndPredatorsExperiment") {
        initializeDataForExperimentPlainWithObstaclesAndPredators(simState, basicBoidCount);
    } else if (experimentId == "defaultSettingsExperiment") {
        initializeDataForExperimentDefault(simState, basicBoidCount);
    } else if (experimentId == "defaultSettings3DExperiment") {
        initializeDataForExperimentDefault3D(simState, basicBoidCount);
    } else {
        throw std::runtime_error("Unknown experiment ID: " + experimentId);
    }
}

void ExperimentLab::initializeDataForExperimentPlain(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    simulator::spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);
}

void ExperimentLab::initializeDataForExperimentPlainWithObstacles(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    simulator::spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    const int obstacleCount = 50;
    simulator::spawnBoids(simState, BoidType::Obstacle, simState.boids.obstacleBoidCount, obstacleCount);
}

void ExperimentLab::initializeDataForExperimentPlainWithObstaclesAndPredators(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    simulator::spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    const int obstacleCount = 50;
    simulator::spawnBoids(simState, BoidType::Obstacle, simState.boids.obstacleBoidCount, obstacleCount);

    // Spawn predator boids
    const int predatorCount = 5;
    simulator::spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);
}

void ExperimentLab::initializeDataForExperimentDefault(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    simulator::spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    Vec3 obstaclePos = { simState.worldX.number() / 2.0f, simState.worldY.number() / 2.0f, 0.0f };
    Vec3 obstacleVel = { 0.0f, 0.0f, 0.0f };
    simulator::spawnBoid(simState, BoidType::Obstacle, obstaclePos, obstacleVel);

    // Spawn predator boids
    const int predatorCount = 1;
    simulator::spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);
}

void ExperimentLab::initializeDataForExperimentDefault3D(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    simulator::spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    Vec3 obstaclePos = { simState.worldX.number() / 2.0f, simState.worldY.number() / 2.0f, 0.0f };
    Vec3 obstacleVel = { 0.0f, 0.0f, 0.0f };
    simulator::spawnBoid(simState, BoidType::Obstacle, obstaclePos, obstacleVel);

    // Spawn predator boids
    const int predatorCount = 1;
    simulator::spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);
}

void ExperimentLab::finalizeExperimentResults(const Config& experimentConfig, std::vector<ExperimentResult>& results) {
    // Example: Output results to console
    for (const ExperimentResult& result : results) {
        std::cout << "Results for version: " << result.version << '\n';
        std::cout << "Boid Count, Average Step Time (ms)\n";
        for (size_t i = 0; i < result.boidCounts.size(); ++i) {
            std::cout << result.boidCounts[i] << ", " << result.avgStepTimesMs[i] << '\n';
        }
        std::cout << '\n';
    }

    // Additional finalization logic (e.g., saving to file) can be added here
}

std::vector<std::string> ExperimentLab::getVersionsToExperimentWith(const Config& experimentConfig) {
    // Get version list from experiment config
    std::vector<std::string> requestedVersions = experimentConfig.get("versionsToRun").enumOptions();

    // Get all available versions
    std::vector<std::string> availableVersions = versionManager.getAvailableVersions();

    // Filter versions to only those available
    std::vector<std::string> versions;
    for (const std::string& version : requestedVersions) {
        bool versionFound = false;
        for (const std::string& availableVersion : availableVersions) {
            if (version == availableVersion) {
                versions.push_back(version);
                versionFound = true;
                break;
            }
        }
        if (!versionFound) {
            std::cerr << "Warning: Requested version '" << version << "' is not available and will be skipped.\n";
        }
    }

    return versions;
}