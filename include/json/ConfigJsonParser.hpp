#pragma once

#include <vector>

#include "nlohmann/json.hpp"
#include "config/Config.hpp"


Config parseConfig(const nlohmann::json& root);
