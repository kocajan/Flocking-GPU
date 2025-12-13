#pragma once
#include "core/WorldState.hpp"
#include "models/IFlockingModel.hpp"

class IBackend {
public:
    virtual ~IBackend() = default;

    virtual void step(
        const IFlockingModel& model,
        const WorldState& in,
        WorldState& out,
        float dt
    ) = 0;
};
