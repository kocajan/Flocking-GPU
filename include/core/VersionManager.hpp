#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>

#include "config/SimConfig.hpp"
#include "config/VersionConfig.hpp"
#include "config/ConfigParameter.hpp"

class VersionManager {
public:
    using VersionId = std::string;

private:
    // Fully materialized SimConfigs per version
    std::unordered_map<VersionId, SimConfig> configs;

public:
    // Version selector parameter (enum)
    ConfigParameter versions;

public:
    explicit VersionManager(const std::vector<VersionConfig>& versions);

    bool hasVersion(const VersionId& version) const;
    std::vector<VersionId> getAvailableVersions() const;

    const SimConfig& getSimConfig(const VersionId& version) const;
};
