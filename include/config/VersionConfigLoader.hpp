#pragma once

#include <vector>
#include <string>

#include "config/Config.hpp"


std::vector<Config> loadConfigs(const std::string& jsonPath);
