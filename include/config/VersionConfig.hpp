#pragma once

#include <string>
#include <vector>

#include "ConfigParameter.hpp"

// Represents one version definition loaded from JSON
struct VersionConfig {
    std::string versionId;
    std::vector<ConfigParameter> parameters;
};
