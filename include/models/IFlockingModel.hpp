#pragma once
#include <string_view>
#include "core/WorldState.hpp"

class UniformGrid; // forward declaration

class IFlockingModel {
public:
    virtual ~IFlockingModel() = default;

    // Grid is guaranteed to be rebuilt before calling step
    virtual void step(
        const WorldState& current,
        WorldState& next,
        const UniformGrid& grid,
        float dt
    ) const = 0;

    // virtual ModelDescriptor descriptor() const = 0;
    virtual bool setParam(std::string_view name, float value) = 0;
};
