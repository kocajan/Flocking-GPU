#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>

#include "config/SimConfig.hpp"
#include "config/VersionConfig.hpp"
#include "config/ConfigParameter.hpp"

class VersionManager {
private:
    // Fully materialized SimConfigs per version
    std::unordered_map<std::string, SimConfig> configs;

public:
    // Version selector parameter (enum)
    ConfigParameter versions;

public:
    explicit VersionManager(const std::vector<VersionConfig>& versions);

    bool hasVersion(const std::string& version) const;
    std::vector<std::string> getAvailableVersions() const;

    const SimConfig& getSimConfig(const std::string& version) const;
};
