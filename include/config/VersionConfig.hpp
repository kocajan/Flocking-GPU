#pragma once

#include <string>
#include <vector>

#include "Config.hpp"
#include "ConfigParameter.hpp"

class VersionConfig : public Config {
public:
    explicit VersionConfig(std::string id = {}) : Config(std::move(id)) {}
};
