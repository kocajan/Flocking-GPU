#pragma once
#include "Config.hpp"

class SimConfig : public Config {
public:
    explicit SimConfig(std::string id = {}) : Config(std::move(id)) {}
};
