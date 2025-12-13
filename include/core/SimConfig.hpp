#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <cassert>

#include "ConfigParameter.hpp"

// ============================================================
// Simulation configuration container
// ============================================================

class SimConfig {
public:
    // --------------------------------------------------------
    // Storage
    // --------------------------------------------------------
    std::vector<ConfigParameter> params;

private:
    // Fast lookup: name -> index in params
    std::unordered_map<std::string, std::size_t> indexByName;

public:
    // ========================================================
    // Construction
    // ========================================================

    SimConfig() = default;

    // Add a parameter (takes ownership)
    void add(ConfigParameter param) {
        // Check whether the name already exists in the config
        assert(indexByName.find(param.name) == indexByName.end());

        indexByName[param.name] = params.size();
        params.push_back(std::move(param));
    }

    // ========================================================
    // Lookup
    // ========================================================

    bool has(const std::string& name) const {
        return indexByName.find(name) != indexByName.end();
    }

    ConfigParameter& get(const std::string& name) {
        auto it = indexByName.find(name);
        assert(it != indexByName.end());
        return params[it->second];
    }

    const ConfigParameter& get(const std::string& name) const {
        auto it = indexByName.find(name);
        assert(it != indexByName.end());
        return params[it->second];
    }

    // ========================================================
    // Typed helpers (ergonomic but optional)
    // ========================================================

    float number(const std::string& name) const {
        return get(name).number();
    }

    bool binary(const std::string& name) const {
        return get(name).binary();
    }

    const std::string& string(const std::string& name) const {
        return get(name).string();
    }

    // Mutable access (for GUI / scripting)
    float& numberRef(const std::string& name) {
        return get(name).number();
    }

    bool& binaryRef(const std::string& name) {
        return get(name).binary();
    }

    std::string& stringRef(const std::string& name) {
        return get(name).string();
    }

    // ========================================================
    // Utilities
    // ========================================================

    void resetAll() {
        for (auto& p : params) {
            p.reset();
        }
    }
};
