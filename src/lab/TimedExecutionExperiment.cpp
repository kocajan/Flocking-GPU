/**
 * \file TimedExecutionExperiment.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of timing aggregation experiment.
 */

#include <cstdio>
#include <filesystem>

#include "lab/TimedExecutionExperiment.hpp"

// -----------------------------------------------------------------------------
//  Experiment lifecycle
// -----------------------------------------------------------------------------

/**
 * \brief Allocate timing result entries for all configured versions.
 *
 * Called once before any experiment runs begin.
 */
void TimedExecutionExperiment::onExperimentStart() {
    const std::vector<std::string> versions =
        experimentConfig.get("versionsToRun").enumOptions();

    for (const std::string& version : versions) {
        auto* entry = new TimedExecutionEntry();
        entry->version = version;
        results[version] = entry;
    }
}

// -----------------------------------------------------------------------------
//  Version lifecycle
// -----------------------------------------------------------------------------

/**
 * \brief Reset timing accumulators and select active result entry.
 *
 * \param[in] version Version identifier for the current run.
 */
void TimedExecutionExperiment::onVersionStart(const std::string& version) {
    currentSamples = 0;
    accumulatedMs = 0.0;
    currentEntry = results[version];
}

/**
 * \brief Record boid count used in this sweep iteration.
 *
 * The count is appended to all version entries to preserve consistent indexing.
 *
 * \param[in] boidCount Number of boids in the current experiment run.
 */
void TimedExecutionExperiment::onBoidNumChangeStart(int boidCount) {
    for (auto& [_, entry] : results) {
        entry->boidCounts.push_back(boidCount);
    }
}

// -----------------------------------------------------------------------------
//  Tick timing collection
// -----------------------------------------------------------------------------

/**
 * \brief Accumulate timing sample for the current version and boid count.
 *
 * \param[in] tick Tick index (unused).
 * \param[in] stepMs Duration of simulation step in milliseconds.
 */
void TimedExecutionExperiment::onTick(int /*tick*/, double stepMs) {
    accumulatedMs += stepMs;
    ++currentSamples;
}

/**
 * \brief Compute and store average tick duration for the finished version run.
 *
 * \param[in] version Version identifier (unused).
 */
void TimedExecutionExperiment::onVersionEnd(const std::string& /*version*/) {
    const double avg =
        (currentSamples > 0)
            ? accumulatedMs / static_cast<double>(currentSamples)
            : 0.0;

    currentEntry->avgStepTimesMs.push_back(avg);
}

// -----------------------------------------------------------------------------
//  Results export
// -----------------------------------------------------------------------------

/**
 * \brief Write timing results to CSV files, one per version.
 *
 * Output format:
 *   BoidCount,AvgStepTimeMs
 */
void TimedExecutionExperiment::onExperimentEnd() {
    const std::string outputDir =
        experimentConfig.string("outputDirPath") +
        "/timed_execution_results";

    std::filesystem::create_directories(outputDir);

    for (const auto& [version, entry] : results) {
        const std::string filename =
            outputDir + "/" + version + "_timed_execution_results.csv";

        FILE* file = std::fopen(filename.c_str(), "w");
        if (file == nullptr) {
            std::fprintf(
                stderr,
                "Error: could not open output file for writing: %s\n",
                filename.c_str()
            );
            continue;
        }

        // CSV header
        std::fprintf(file, "BoidCount,AvgStepTimeMs\n");

        // Rows
        for (std::size_t i = 0; i < entry->boidCounts.size(); ++i) {
            std::fprintf(
                file,
                "%d,%.6f\n",
                entry->boidCounts[i],
                entry->avgStepTimesMs[i]
            );
        }

        std::fclose(file);
    }
}
