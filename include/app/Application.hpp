#pragma once

#include <string>
#include <vector>
#include <filesystem>

#include "gui/GUI.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"
#include "core/VersionManager.hpp"

class Application {
public:
    Application(const std::string& configDirPath);

    bool isInitialized();

    void run();

private:
    bool initialized = false;

    GUI gui;

    VersionManager versionManager;

    Config simConfig;
    SimState simState;

    std::string currentVersion;
};
