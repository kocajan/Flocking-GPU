#pragma once
#include "core/Types.hpp"

enum class Dimensionality : int {
    D2 = 2,
    D3 = 3
};

struct SimBounds {
    Vec3 min;
    Vec3 max;
};

struct SimConfig {
    SimBounds bounds;
    Dimensionality dims = Dimensionality::D2;

    // First implementation: clamp to bounds (no wrap yet).
    bool wrap = false;
};
