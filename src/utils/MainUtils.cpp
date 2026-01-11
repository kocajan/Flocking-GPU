/**
 * \file MainUtils.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of command-line parsing and config path utilities.
 */

#include <cstdlib>
#include <iostream>

#include "utils/MainUtils.hpp"


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

bool isHelpMode(std::string_view mode) {
    return (mode == "help" || mode == "-h" || mode == "--help");
}

std::filesystem::path resolveConfigPath(
    int argc,
    char** argv,
    int indexFallback
) {
    std::filesystem::path path =
        (argc > indexFallback ? argv[indexFallback] : "cfg");

    if (!std::filesystem::exists(path)) {
        std::cerr << "Error: config path does not exist: "
                  << path.string() << '\n';
        printUsage();
        std::exit(1);
    }

    return path;
}

std::optional<CommandLine> parseArgs(int argc, char** argv) {
    if (argc <= 1) {
        return std::nullopt;
    }

    const std::string_view modeArg = argv[1];

    if (isHelpMode(modeArg)) {
        return CommandLine{ Mode::Help, {} };
    }

    const auto configPath = resolveConfigPath(argc, argv);

    if (modeArg == "run") {
        return CommandLine{ Mode::Run, configPath };
    }

    if (modeArg == "experiment") {
        return CommandLine{ Mode::Experiment, configPath };
    }

    std::cerr << "Unknown mode: " << modeArg << "\n\n";
    return std::nullopt;
}
