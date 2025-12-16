#pragma once

#include "core/Types.hpp"

struct Boid {
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;

    BoidType type;

    Color color;
    float radius;
};
