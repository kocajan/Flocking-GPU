#pragma once
#include <cstdint>

struct Vec3 {
    float x, y, z;
};

enum class BoidType : uint8_t {
    Bird,
    Predator,
    Obstacle,
    Custom
};

struct Color {
    float r;
    float g;
    float b;
    float a;
};
