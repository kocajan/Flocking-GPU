#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>

#include "config/Config.hpp"
#include "config/ConfigParameter.hpp"

class VersionManager {
private:
    // Fully materialized Configs per version
    std::unordered_map<std::string, Config> configs;

public:
    // Version selector parameter (enum)
    ConfigParameter versions;

public:
    explicit VersionManager(const std::vector<Config>& versions);

    bool hasVersion(const std::string& version) const;
    std::vector<std::string> getAvailableVersions() const;

    const Config& getSimConfig(const std::string& version) const;
};
