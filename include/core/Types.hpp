#pragma once

#include <cstdint>
#include <vector>
#include <variant>
#include <string>

// Basic helper types
struct Vec3 {
    float x, y, z;
};

struct Color {
    int r;
    int g;
    int b;
    int a;
};

// In-Simulation interaction types
enum class InteractionType : uint8_t {
    Repel,
    Attract,
    Empty
};

struct Interaction {
    InteractionType type;
    Vec3 point;
};

// Boid types
enum class BoidType : uint8_t {
    Basic,
    Predator,
    Obstacle,
    Empty
};

struct Boid {
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;

    BoidType type;

    std::string color;
    float radius;
};

// ############# ConfigParameter related types #############
// Parameter types
enum class ParamType {
    Number,
    Binary,
    String,
    Enum
};

// Preferred rendering style for GUI
enum class ParamRender {
    Checkbox,
    Button,
    ToggleButton,
    Slider,         
    Drag,
    Input
};

// Range types for ConfigParameter
// ---------- Number ----------
struct NumberRange {
    float min = 0.0f;
    float max = 1.0f;
    float step = 0.01f;
};

// ---------- Binary ----------
struct BinaryRange {
    bool allowTrue  = true;
    bool allowFalse = true;
};

// ---------- String ----------
struct StringRange {
    bool freeText = true;                      // allow arbitrary input
    std::vector<std::string> options;          // optional allowed values
};

// ---------- Enum ----------
struct EnumRange {
    std::vector<std::string> options;
};

// Define unified value container
using ParamValue = std::variant<
    bool,
    int,
    float,
    std::string
>;

// Define custom range placeholder
using ParamRange = std::variant<
    NumberRange,
    BinaryRange,
    StringRange,
    EnumRange
>;
