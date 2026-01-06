/**
 * \file
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Entry point of the application. Selects execution mode and runs either the
 *        main application or the experiment laboratory.
 */

#include <string>
#include <iostream>
#include <filesystem>

#include "app/Application.hpp"
#include "lab/ExperimentLab.hpp"
#include "utils/mainUtils.hpp"

/**
 * \brief Program entry point.
 *
 * Supported modes:
 * - "run" — runs the main application
 * - "experiment" — runs experiment laboratory workflows
 * - "help", "-h", "--help" — shows usage information
 *
 * \param argc Number of command-line arguments.
 * \param argv Array of command-line argument strings.
 * \return Exit status code (0 on success, non-zero on error).
 */
int main(int argc, char** argv) {
    if (argc <= 1) {
        printUsage();
        return 1;
    }

    const std::string mode = argv[1];

    if (isHelpMode(mode)) {
        printUsage();
        return 0;
    }

    if (mode == "run") {
        /// Path to configuration directory for application run mode
        const std::string configPath = resolveConfigPath(argc, argv);

        Application app(configPath);
        app.run();
        return 0;

    } else if (mode == "experiment") {
        /// Path to configuration directory for experiment mode
        const std::string configPath = resolveConfigPath(argc, argv);

        ExperimentLab laboratory(configPath);
        laboratory.runExperiments(configPath + "/experiments");
        return 0;
    }

    std::cerr << "Unknown mode: " << mode << '\n';
    printUsage();
    return 1;
}
