#pragma once

#include <vector>
#include <string>

#include "VersionConfig.hpp"

// Loads version configurations from JSON file and returns parsed configs
std::vector<VersionConfig>
loadVersionConfigs(const std::string& jsonPath);
