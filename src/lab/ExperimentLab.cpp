#include <chrono>
#include <iostream>

#include "lab/ExperimentLab.hpp"

#include "config/ConfigLoader.hpp"
#include "config/MultipleConfigLoader.hpp"
#include "simulator/SimulationStep.hpp"
#include "simulator/SimulatorHelpers.hpp"
#include "lab/Experiment.hpp"
#include "lab/TimedExecutionExperiment.hpp"
#include "lab/RecordBoidFramesExperiment.hpp"


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

        // Create timed execution and render frames experiment instances
        TimedExecutionExperiment experimentTimed(adjustedSimState, adjustedSimConfig, experimentConfig);
        RecordBoidFramesExperiment experimentRecord(adjustedSimState, adjustedSimConfig, experimentConfig);

        // Run both experiments
        printf("\n - Running Timed Execution Experiment...\n");
        runExperimentScenario(experimentTimed);
        printf("\n - Running Record Boid Frames Experiment...\n");
        runExperimentScenario(experimentRecord);

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

void ExperimentLab::runExperimentScenario(Experiment& experiment) {
    // Get experiment parameters from the experiment config
    const int totalTicks  = static_cast<int>(experiment.experimentConfig.number("totalTicks"));
    const int warmupTicks = static_cast<int>(experiment.experimentConfig.number("warmupTicks"));
    const int intialStateTicks = static_cast<int>(experiment.experimentConfig.number("initialStateTicks"));

    std::vector<std::string> versions = getVersionsToExperimentWith(experiment.experimentConfig);
    NumberRange boidRange = experiment.experimentConfig.get("numBoids").numberRange();

    // Run experiment pipeline
    experiment.onExperimentStart();
    for (int numBoids = boidRange.min; numBoids <= boidRange.max; numBoids += boidRange.step) {
        printf("    - Running with %d boids...\n", numBoids);

        // Initialize data for this experiment run
        initializeDataForExperiment(experiment.experimentConfig, experiment.simState,numBoids);
        
        // First, run the simulation to get the desired SimState - we want to measure 'steady state' performance
        experiment.simState.version.string() = "parallelNaive";
        for (int tick = 0; tick < intialStateTicks; ++tick) {
            simulationStep(experiment.simState, experiment.simConfig);
        }
        const SimState simStatePersistent = experiment.simState;

        experiment.onBoidNumChangeStart(numBoids);

        for (const std::string& version : versions) {
            printf("  - Starting version: %s\n", version.c_str());
            experiment.simState = simStatePersistent;
            experiment.simState.version.string() = version;

            experiment.onVersionStart(version);
            // Run simulation steps
            for (int tick = 0; tick < totalTicks; ++tick) {

                auto t0 = std::chrono::steady_clock::now();
                simulationStep(experiment.simState, experiment.simConfig);
                auto t1 = std::chrono::steady_clock::now();

                if (tick < warmupTicks)
                    continue;

                std::chrono::duration<double,std::milli> dt = t1 - t0;

                experiment.onTick(tick, dt.count());
            }
            experiment.onVersionEnd(version);
        }
        experiment.onBoidNumChangeEnd(numBoids);
    }
    experiment.onExperimentEnd();
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
    simState.boids.obstacleBoidCount++;

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
    simState.boids.obstacleBoidCount++;
    
    // Spawn predator boids
    const int predatorCount = 1;
    simulator::spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);

    // Set SimState to 3D mode
    simState.dimensions.string() = "3D";
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