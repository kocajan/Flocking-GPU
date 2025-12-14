#pragma once

#include <string>
#include <unordered_map>
#include <cassert>

#include "config/SimConfig.hpp"
#include "config/VersionConfig.hpp"

class VersionManager {
public:
    using VersionId = std::string;

private:
    // Fully materialized SimConfigs per version
    std::unordered_map<VersionId, SimConfig> configs;

public:
    // --------------------------------------------------------
    // Construction
    // --------------------------------------------------------

    // Takes parsed version configs and builds SimConfigs
    explicit VersionManager(const std::vector<VersionConfig>& versions);

    // --------------------------------------------------------
    // Queries
    // --------------------------------------------------------

    bool hasVersion(const VersionId& version) const;
    
    std::vector<VersionId> getAvailableVersions() const;

    const SimConfig& getSimConfig(const VersionId& version) const;
};
