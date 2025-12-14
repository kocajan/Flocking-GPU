#pragma once

#include <string>

#include "config/SimStateConfig.hpp"

// Loads sim-state configuration from JSON file
SimStateConfig loadSimStateConfig(const std::string& jsonPath);
