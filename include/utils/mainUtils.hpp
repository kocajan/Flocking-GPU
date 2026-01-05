/**
 * \file mainUtils.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Command-line utilities for selecting application mode and resolving configuration paths.
 *
 * Provides parsing helpers for CLI arguments and basic usage output.
 */

#pragma once

#include <string>
#include <filesystem>
#include <optional>

/**
 * \enum Mode
 * \brief Execution mode selected from command-line arguments.
 */
enum class Mode {
    Run,        ///< Run the main application.
    Experiment, ///< Run experiment laboratory mode.
    Help        ///< Show help / usage output.
};

/**
 * \struct CommandLine
 * \brief Parsed command-line arguments.
 */
struct CommandLine {
    Mode mode;                        ///< Selected execution mode.
    std::filesystem::path configPath; ///< Resolved configuration directory path.
};

/**
 * \brief Print command-line usage information to stderr.
 */
void printUsage();

/**
 * \brief Check whether the mode string represents a help request.
 *
 * \param[in] mode Command-line mode token.
 * \return true if help mode is requested, false otherwise.
 */
bool isHelpMode(std::string_view mode);

/**
 * \brief Resolve configuration directory path from command-line arguments.
 *
 * Uses the argument at \p indexFallback if available; otherwise defaults to
 * the relative directory `"cfg"`. Terminates the program if the path does not exist.
 *
 * \param[in] argc Number of command-line arguments.
 * \param[in] argv Array of command-line argument strings.
 * \param[in] indexFallback Index of argument to interpret as config path (default 2).
 * \return Resolved configuration path.
 */
std::filesystem::path resolveConfigPath(
    int argc,
    char** argv,
    int indexFallback = 2
);

/**
 * \brief Parse command-line arguments into structured form.
 *
 * \param[in] argc Number of command-line arguments.
 * \param[in] argv Array of command-line argument strings.
 * \return Parsed command-line data, or std::nullopt if the arguments are invalid.
 */
std::optional<CommandLine> parseArgs(int argc, char** argv);
