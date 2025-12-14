#include "config/Config.hpp"

#include <cassert>

Config::Config() = default;
Config::~Config() = default;

void Config::add(ConfigParameter param) {
    assert(indexByName.find(param.name) == indexByName.end());
    indexByName[param.name] = params.size();
    params.push_back(std::move(param));
}

bool Config::has(const std::string& name) const {
    return indexByName.find(name) != indexByName.end();
}

ConfigParameter& Config::get(const std::string& name) {
    auto it = indexByName.find(name);
    assert(it != indexByName.end());
    return params[it->second];
}

const ConfigParameter& Config::get(const std::string& name) const {
    auto it = indexByName.find(name);
    assert(it != indexByName.end());
    return params[it->second];
}

float Config::number(const std::string& name) const {
    return get(name).number();
}

bool Config::binary(const std::string& name) const {
    return get(name).binary();
}

const std::string& Config::string(const std::string& name) const {
    return get(name).string();
}

float& Config::numberRef(const std::string& name) {
    return get(name).number();
}

bool& Config::binaryRef(const std::string& name) {
    return get(name).binary();
}

std::string& Config::stringRef(const std::string& name) {
    return get(name).string();
}

void Config::resetAll() {
    for (auto& p : params) {
        p.reset();
    }
}

std::vector<ConfigParameter>& Config::getParameters() {
    return params;
}

const std::vector<ConfigParameter>& Config::getParameters() const {
    return params;
}
