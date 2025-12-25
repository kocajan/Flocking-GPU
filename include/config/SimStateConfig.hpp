#pragma once
#include "Config.hpp"

class SimStateConfig : public Config {
public:
    explicit SimStateConfig(std::string id = {}) : Config(std::move(id)) {}
};
