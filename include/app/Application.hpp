/**
 * \file Application.hpp
 * \author
 * \date
 * \brief Main GUI application wrapper for the flocking simulation.
 *
 * Encapsulates initialization, configuration loading,
 * simulation state management, and the main GUI loop.
 */

#pragma once

#include <string>
#include <vector>
#include <filesystem>

#include "gui/GUI.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"
#include "core/VersionManager.hpp"

/**
 * \class Application
 * \brief High-level application controller.
 *
 * Responsible for:
 * - loading version and simulation configuration
 * - initializing GUI platform and ImGui
 * - running the main simulation loop
 * - handling version switching and reset operations
 */
class Application {
public:

    /**
     * \brief Construct application instance and perform initialization.
     *
     * Loads configuration, initializes GUI platform, ImGui,
     * and prepares simulation state for execution.
     *
     * \param[in] configDirPath Path to configuration directory.
     */
    Application(const std::string& configDirPath);

    /**
     * \brief Check whether initialization succeeded.
     *
     * \return true if the application is ready to run, false otherwise.
     */
    bool isInitialized();

    /**
     * \brief Run the main GUI simulation loop.
     *
     * No effect if initialization failed.
     */
    void run();

private:
    bool initialized = false; ///< Indicates whether initialization succeeded.

    GUI gui; ///< GUI subsystem.

    VersionManager versionManager; ///< Version configuration manager.

    Config simConfig; ///< Active simulation configuration for current version.
    SimState simState; ///< Mutable simulation state.

    std::string currentVersion; ///< Currently active simulation version identifier.
};
