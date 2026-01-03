#include <string>
#include <iostream>
#include <filesystem>

#include "app/Application.hpp"
#include "lab/ExperimentLab.hpp"

namespace {

void printUsage() {
    std::cerr
        << "Flocking Simulation — run modes\n\n"
        << "Usage:\n"
        << "  flocking run <cfg_dir>\n"
        << "  flocking experiment <cfg_dir>\n"
        << "  flocking help\n\n"
        << "Modes:\n"
        << "  run         Launch interactive GUI simulation.\n"
        << "              <cfg_dir> defaults to ./cfg if omitted.\n\n"
        << "  experiment  Run headless experiment pipeline.\n"
        << "              Uses the same config + simulation core,\n"
        << "              but without GUI, input or pause logic.\n\n"
        << "  help, -h, --help\n"
        << "              Show this help message.\n";
}

std::string resolveConfigPathOrFail(int argc, char** argv, int indexFallback = 2) {
    std::string configPath = (argc > indexFallback ? argv[indexFallback] : "cfg");

    if (!std::filesystem::exists(configPath)) {
        std::cerr << "Error: config path does not exist: " << configPath << '\n';
        printUsage();
        std::exit(1);
    }

    return configPath;
}

bool isHelpMode(const std::string& mode) {
    return (mode == "help" || mode == "-h" || mode == "--help");
}

} // namespace


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
            const std::string configPath = resolveConfigPathOrFail(argc, argv);

            Application app(configPath);
            app.run();
            return 0;
    }

    if (mode == "experiment") {
        const std::string configPath = resolveConfigPathOrFail(argc, argv);

        ExperimentLab laboratory(configPath);
        laboratory.runExperiments(configPath + "/experiments");
        return 0;
    }

    std::cerr << "Unknown mode: " << mode << '\n';
    printUsage();
    return 1;
}
