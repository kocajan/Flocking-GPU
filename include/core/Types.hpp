#pragma once
#include <cstdint>

struct Vec3 {
    float x, y, z;
};

enum class BoidType : uint8_t {
    Basic,
    Predator,
    Obstacle,
    Empty
};

struct Color {
    float r;
    float g;
    float b;
    float a;
};

enum class InteractionType : uint8_t {
    Repel,
    Attract,
    Empty
};

struct Interaction {
    InteractionType type;
    Vec3 point;
};
