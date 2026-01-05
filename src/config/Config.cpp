/**
 * \file Config.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of configuration container and parameter grouping utilities.
 */

#include <cassert>
#include <iostream>

#include "config/Config.hpp"


Config::~Config() = default;

// Config ID management
const std::string& Config::getConfigId() const {
    return configId;
}

void Config::setConfigId(std::string id) {
    configId = std::move(id);
}

// Parameter management
void Config::add(ConfigParameter param, const std::string& groupName) {
    assert(indexByName.find(param.name) == indexByName.end());

    std::size_t idx = params.size();

    indexByName[param.name] = idx;
    params.push_back(std::move(param));

    // Map parameter -> group
    paramGroupByName[params[idx].name] = groupName;

    // First time we see the group -> record ordering
    if (groupParams.find(groupName) == groupParams.end()) {
        groupOrder.push_back(groupName);
        groupParams[groupName] = {};
    }

    // Append parameter name to group list
    groupParams[groupName].push_back(params[idx].name);
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
    if (it == indexByName.end()) {
        std::cerr << "Config::get(): parameter not found: " << name << "\n";
        std::abort();
    }
    return params[it->second];
}

std::vector<ConfigParameter>& Config::getParameters() {
    return params;
}

const std::vector<ConfigParameter>& Config::getParameters() const {
    return params;
}

// Special accessors for values of the ConfigParameters
float Config::number(const std::string& name) const {
    return get(name).number();
}

bool Config::binary(const std::string& name) const {
    return get(name).binary();
}

const std::string& Config::string(const std::string& name) const {
    return get(name).string();
}

// Special accessors for references to values of the ConfigParameters
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
