#pragma once

#include "core/Types.hpp"

struct Boid {
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;

    BoidType type;

    float radius;
    float maxSpeed;
    float maxForce;

    float alignmentWeight;
    float cohesionWeight;
    float separationWeight;

    Color color;
};
