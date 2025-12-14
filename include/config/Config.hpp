#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include "ConfigParameter.hpp"

class Config {
public:
    Config();
    virtual ~Config();

    void add(ConfigParameter param);

    bool has(const std::string& name) const;

    ConfigParameter& get(const std::string& name);
    const ConfigParameter& get(const std::string& name) const;

    float number(const std::string& name) const;
    bool binary(const std::string& name) const;
    const std::string& string(const std::string& name) const;

    float& numberRef(const std::string& name);
    bool& binaryRef(const std::string& name);
    std::string& stringRef(const std::string& name);

    virtual void resetAll();

    std::vector<ConfigParameter>& getParameters();
    const std::vector<ConfigParameter>& getParameters() const;

protected:
    std::vector<ConfigParameter> params;
    std::unordered_map<std::string, std::size_t> indexByName;
};
