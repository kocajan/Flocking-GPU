#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include "config/ConfigParameter.hpp"

class Config {
public:
    explicit Config(std::string id = {}) : configId(std::move(id)) {}
    virtual ~Config();

    // Config ID
    const std::string& getConfigId() const;
    void setConfigId(std::string id);

    // Parameter lookup API
    void add(ConfigParameter param, const std::string& groupName);
    bool has(const std::string& name) const;
    ConfigParameter& get(const std::string& name);
    const ConfigParameter& get(const std::string& name) const;
    std::vector<ConfigParameter>& getParameters();
    const std::vector<ConfigParameter>& getParameters() const;

    // Value helpers
    float number(const std::string& name) const;
    bool binary(const std::string& name) const;
    const std::string& string(const std::string& name) const;

    float& numberRef(const std::string& name);
    bool& binaryRef(const std::string& name);
    std::string& stringRef(const std::string& name);

    // Reset to defaults
    virtual void resetAll();

    // Grouping metadata

    // Parameter name -> group name
    const std::unordered_map<std::string, std::string>& getParamGroups() const {
        return paramGroupByName;
    }

    // Group name -> parameter names
    const std::unordered_map<std::string, std::vector<std::string>>& getGroupParams() const {
        return groupParams;
    }

    // Ordered list of group names
    const std::vector<std::string>& getGroupOrder() const {
        return groupOrder;
    }

private:
    std::string configId;
    std::vector<ConfigParameter> params;
    std::unordered_map<std::string, std::size_t> indexByName;

    // Grouping structures
    std::unordered_map<std::string, std::string> paramGroupByName;
    std::unordered_map<std::string, std::vector<std::string>> groupParams;
    std::vector<std::string> groupOrder;
};
