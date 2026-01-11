/**
 * \file ExperimentLab.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of experiment setup and execution framework.
 */

#include <chrono>
#include <iostream>

#include "lab/ExperimentLab.hpp"

#include "config/ConfigLoader.hpp"
#include "config/MultipleConfigLoader.hpp"
#include "simulator/SimulationStep.hpp"
#include "simulator/BoidPopulationRegulationUtils.hpp"
#include "lab/Experiment.hpp"
#include "lab/TimedExecutionExperiment.hpp"
#include "lab/RecordBoidFramesExperiment.hpp"

// -----------------------------------------------------------------------------
//  ExperimentLab intialization
// -----------------------------------------------------------------------------

ExperimentLab::ExperimentLab(const std::string& configDirPath)
    : versionManager(loadConfigs(configDirPath + "/versions")),
      defaultSimState(loadConfig(configDirPath + "/initialSimulationState.json"), versionManager.versions) {
    
    // Load default simulation config for the default version
    std::string currentVersion = defaultSimState.version.string();
    defaultSimConfig = versionManager.getSimConfig(currentVersion);

    // Mark as initialized
    initialized = true;
}

// -----------------------------------------------------------------------------
//  Initialization status check
// -----------------------------------------------------------------------------

bool ExperimentLab::isInitialized() {
    return initialized;
}

// -----------------------------------------------------------------------------
//  Experiment execution
// -----------------------------------------------------------------------------

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

/**
 * \brief Execute a single experiment scenario across boid counts and versions.
 *
 * Performs:
 * - warmup and steady-state preparation
 * - version iteration
 * - tick timing + callbacks to experiment hooks
 *
 * \param[in,out] experiment Experiment instance implementing hook callbacks.
 */
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

        // Notify experiment of boid count change and let it react
        experiment.onBoidNumChangeStart(numBoids);

        for (const std::string& version : versions) {
            printf("  - Starting version: %s\n", version.c_str());

            // Reset sim state to the persistent state and set version
            experiment.simState = simStatePersistent;
            experiment.simState.version.string() = version;

            // Notify experiment of version start and let it react
            experiment.onVersionStart(version);
            
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

// -----------------------------------------------------------------------------
//  Simulation config and state adjustment helpers
// -----------------------------------------------------------------------------

/**
 * \brief Apply experiment-specific overrides to the default simulation state.
 *
 * Parameters present in the experiment config group "SimState" replace
 * corresponding values in the copied default state.
 *
 * \param[in] experimentConfig Experiment configuration.
 * \return Adjusted simulation state instance.
 */
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

/**
 * \brief Apply experiment-specific overrides to the default simulation config.
 *
 * Parameters present in the experiment config group "SimConfig" replace
 * corresponding values in the copied default configuration.
 *
 * \param[in] experimentConfig Experiment configuration.
 * \return Adjusted simulation configuration instance.
 */
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

// -----------------------------------------------------------------------------
//  Version selection helper
// -----------------------------------------------------------------------------

/**
 * \brief Resolve list of versions to run for a given experiment config.
 *
 * Filters requested versions against the versions available in the
 * VersionManager and logs warnings for missing entries.
 *
 * \param[in] experimentConfig Experiment configuration.
 * \return List of valid version identifiers to run.
 */
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

// -----------------------------------------------------------------------------
//  Experiment data initialization routines
// -----------------------------------------------------------------------------

/**
 * \brief Initialize simulation entities according to the selected experiment ID.
 *
 * Dispatches to a specific initialization routine based on the experiment
 * scenario identifier.
 *
 * \param[in] experimentConfig Experiment configuration.
 * \param[in,out] simState Simulation state to initialize.
 * \param[in] requestedBasicBoidCount Number of basic boids to spawn.
 */
void ExperimentLab::initializeDataForExperiment(const Config& experimentConfig, SimState& simState, int requestedBasicBoidCount) {
    // Get current experiment ID
    const std::string& experimentId = experimentConfig.getConfigId();

    if (experimentId == "plainExperiment") {
        initializeDataForExperimentPlain(simState, requestedBasicBoidCount);
    } else if (experimentId == "plainWithObstaclesExperiment") {
        initializeDataForExperimentPlainWithObstacles(simState, requestedBasicBoidCount);
    } else if (experimentId == "plainWithObstaclesAndPredatorsExperiment") {
        initializeDataForExperimentPlainWithObstaclesAndPredators(simState, requestedBasicBoidCount);
    } else if (experimentId == "defaultSettingsExperiment") {
        initializeDataForExperimentDefault(simState, requestedBasicBoidCount);
    } else if (experimentId == "defaultSettings3DExperiment") {
        initializeDataForExperimentDefault3D(simState, requestedBasicBoidCount);
    } else {
        throw std::runtime_error("Unknown experiment ID: " + experimentId);
    }
}

/**
 * \brief Initialize plain experiment layout with only basic boids.
 *
 * Clears existing entities and spawns the requested number of basic boids.
 *
 * \param[in,out] simState Simulation state to modify.
 * \param[in] requestedBasicBoidCount Number of basic boids to spawn.
 */
void ExperimentLab::initializeDataForExperimentPlain(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);
}

/**
 * \brief Initialize plain experiment with static obstacle boids.
 *
 * Spawns basic boids and a fixed number of obstacle boids.
 *
 * \param[in,out] simState Simulation state to modify.
 * \param[in] requestedBasicBoidCount Number of basic boids to spawn.
 */
void ExperimentLab::initializeDataForExperimentPlainWithObstacles(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    const int obstacleCount = 50;
    spawnBoids(simState, BoidType::Obstacle, simState.boids.obstacleBoidCount, obstacleCount);
}

/**
 * \brief Initialize experiment with obstacles and predator boids.
 *
 * Spawns basic, obstacle, and predator entities.
 *
 * \param[in,out] simState Simulation state to modify.
 * \param[in] requestedBasicBoidCount Number of basic boids to spawn.
 */
void ExperimentLab::initializeDataForExperimentPlainWithObstaclesAndPredators(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    const int obstacleCount = 50;
    spawnBoids(simState, BoidType::Obstacle, simState.boids.obstacleBoidCount, obstacleCount);

    // Spawn predator boids
    const int predatorCount = 5;
    spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);
}

/**
 * \brief Initialize default experiment layout (2D) with center obstacle and predator.
 *
 * Spawns requested basic boids, one central obstacle, and one predator boid.
 *
 * \param[in,out] simState Simulation state to modify.
 * \param[in] requestedBasicBoidCount Number of basic boids to spawn.
 */
void ExperimentLab::initializeDataForExperimentDefault(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    Vec3 obstaclePos = { simState.worldX.number() / 2.0f, simState.worldY.number() / 2.0f, 0.0f };
    Vec3 obstacleVel = { 0.0f, 0.0f, 0.0f };
    spawnBoid(simState, BoidType::Obstacle, obstaclePos, obstacleVel);
    simState.boids.obstacleBoidCount++;

    // Spawn predator boids
    const int predatorCount = 1;
    spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);
}

/**
 * \brief Initialize default 3D experiment layout.
 *
 * Same as default 2D layout but enables 3D simulation mode.
 *
 * \param[in,out] simState Simulation state to modify.
 * \param[in] requestedBasicBoidCount Number of basic boids to spawn.
 */
void ExperimentLab::initializeDataForExperimentDefault3D(SimState& simState, int requestedBasicBoidCount) {
    // Get simState boids
    Boids& b = simState.boids;

    // Reset boids
    b.clear();

    // Spawn basic boids
    spawnBoids(simState, BoidType::Basic, simState.boids.basicBoidCount, requestedBasicBoidCount);

    // Spawn obstacle boids
    Vec3 obstaclePos = { simState.worldX.number() / 2.0f, simState.worldY.number() / 2.0f, simState.worldZ.number() / 2.0f };
    Vec3 obstacleVel = { 0.0f, 0.0f, 0.0f };
    spawnBoid(simState, BoidType::Obstacle, obstaclePos, obstacleVel);
    simState.boids.obstacleBoidCount++;
    
    // Spawn predator boids
    const int predatorCount = 1;
    spawnBoids(simState, BoidType::Predator, simState.boids.predatorBoidCount, predatorCount);

    // Set SimState to 3D mode
    simState.dimensions.string() = "3D";
}
