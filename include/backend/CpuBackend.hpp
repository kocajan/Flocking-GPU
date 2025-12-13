#pragma once
#include "backend/IBackend.hpp"
#include "core/SimConfig.hpp"

class CpuBackend final : public IBackend {
public:
    explicit CpuBackend(const SimConfig& cfg, float cellSize);

    void step(
        const IFlockingModel& model,
        const WorldState& in,
        WorldState& out,
        float dt
    ) override;

// private:
//     UniformGrid grid_;
};
