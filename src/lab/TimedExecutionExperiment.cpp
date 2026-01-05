#include <cstdio>
#include <filesystem>

#include "lab/TimedExecutionExperiment.hpp"


void TimedExecutionExperiment::onExperimentStart() {
    // Initialize results for each version (dictionary)
    const std::vector<std::string> versions = experimentConfig.get("versionsToRun").enumOptions();
    for (const std::string& version : versions) {
        TimedExecutionEntry* entry = new TimedExecutionEntry();
        entry->version = version;
        results[version] = entry;
    }
}


void TimedExecutionExperiment::onVersionStart(const std::string& version) {
    currentSamples = 0;
    accumulatedMs = 0.0;

    currentEntry = results[version];
}

void TimedExecutionExperiment::onBoidNumChangeStart(int boidCount) {
    // Save current boid count for all versions
    for (auto& [version, entry] : results) {
        entry->boidCounts.push_back(boidCount);
    }
}

void TimedExecutionExperiment::onTick(int /*tick*/, double stepMs) {
    accumulatedMs += stepMs;
    ++currentSamples;
}

void TimedExecutionExperiment::onVersionEnd(const std::string& /*version*/) {
    double avg = (currentSamples > 0) ? accumulatedMs / static_cast<double>(currentSamples) : 0.0;

    currentEntry->avgStepTimesMs.push_back(avg);
}

void TimedExecutionExperiment::onExperimentEnd() {
    // Base output directory from config
    const std::string outputDir = experimentConfig.string("outputDirPath") + "/timed_execution_results";

    // Ensure directory exists
    std::filesystem::create_directories(outputDir);

    // For each version, write results to CSV file
    for (const auto& [version, entry] : results) {
        const std::string filename = outputDir + "/" + version + "_timed_execution_results.csv";

        FILE* file = std::fopen(filename.c_str(), "w");
        if (file == nullptr) {
            std::fprintf(stderr,
                         "Error: could not open output file for writing: %s\n",
                         filename.c_str());
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
