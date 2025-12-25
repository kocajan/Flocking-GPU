#pragma once

#include <vector>
#include <string>

#include "config/Config.hpp"

std::vector<Config> loadVersionConfigs(const std::string& jsonPath);
