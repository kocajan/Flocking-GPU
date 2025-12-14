#pragma once

#include <vector>

#include "config/VersionConfig.hpp"
#include <nlohmann/json.hpp>

std::vector<VersionConfig>
parseVersionConfigs(const nlohmann::json& root);
