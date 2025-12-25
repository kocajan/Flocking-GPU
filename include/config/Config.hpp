#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include "config/ConfigParameter.hpp"


class Config {
public:
    // Constructor / destructor
    explicit Config(std::string id = {}) : configId(std::move(id)) {}
    virtual ~Config();

    // Config ID management
    const std::string& getConfigId() const;
    void setConfigId(std::string id);

    // Parameter management
    void add(ConfigParameter param);
    bool has(const std::string& name) const;
    ConfigParameter& get(const std::string& name);
    const ConfigParameter& get(const std::string& name) const;
    std::vector<ConfigParameter>& getParameters();
    const std::vector<ConfigParameter>& getParameters() const;

    // Special accessors for values of the ConfigParameters
    float number(const std::string& name) const;
    bool binary(const std::string& name) const;
    const std::string& string(const std::string& name) const;

    // Special accessors for references to values of the ConfigParameters
    float& numberRef(const std::string& name);
    bool& binaryRef(const std::string& name);
    std::string& stringRef(const std::string& name);

    // Reset all parameters to their default values
    virtual void resetAll();

private:
    std::string configId;
    std::vector<ConfigParameter> params;
    std::unordered_map<std::string, std::size_t> indexByName;
};
