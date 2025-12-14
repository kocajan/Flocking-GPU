#pragma once

#include <vector>

#include "config/SimStateConfig.hpp"
#include <nlohmann/json.hpp>

SimStateConfig parseSimStateConfig(const nlohmann::json& root);