#pragma once
#include <vector>
#include <cstddef>
#include "core/Types.hpp"

enum class AgentType : uint8_t {
    Bird,
    Predator,
    Prey
};

struct WorldState {
    std::vector<float> px, py, pz;
    std::vector<float> vx, vy, vz;
    std::vector<AgentType> type;

    size_t size() const {
        return px.size();
    }

    void resize(size_t n) {
        px.resize(n); py.resize(n); pz.resize(n);
        vx.resize(n); vy.resize(n); vz.resize(n);
        type.resize(n);
    }
};
