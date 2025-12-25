#pragma once

#include <vector>
#include <nlohmann/json.hpp>

#include "config/ConfigParameter.hpp"


ConfigParameter parseParameter(const nlohmann::json& j);
