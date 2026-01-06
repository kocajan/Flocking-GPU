/**
 * \file Types.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Core math, interaction, boid, configuration, and parameter range types used across the simulation.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <variant>
#include <chrono>

// ----------------------------------------------------------------------------
//  Basic math + color types
// ----------------------------------------------------------------------------

/**
 * \struct Vec3
 * \brief 3D vector with single-precision components.
 */
struct Vec3 {
    float x;
    float y;
    float z;
};

/**
 * \struct Color
 * \brief RGBA color in integer channel representation.
 */
struct Color {
    int r;
    int g;
    int b;
    int a;
};

// ----------------------------------------------------------------------------
//  Boid types
// ----------------------------------------------------------------------------

/**
 * \enum BoidType
 * \brief Type identifier for boids in mixed populations.
 */
enum class BoidType : uint8_t {
    Basic,    ///< Standard flocking boid.
    Predator, ///< Predator boid.
    Obstacle, ///< Static obstacle boid.
    Empty     ///< No boid / unused entry.
};

// ----------------------------------------------------------------------------
//  Interaction types (from GUI perspective)
// ----------------------------------------------------------------------------

/**
 * \enum MouseInteractionType
 * \brief Type of mouse interaction mapped to world coordinates.
 */
enum class MouseInteractionType : uint8_t {
    LeftClickOnWorld,  ///< Left mouse click drag.
    RightClickOnWorld  ///< Right mouse click drag.
};

/**
 * \struct MouseInteractionEvent
 * \brief Runtime mouse interaction state with timestamps and world position.
 */
struct MouseInteractionEvent {
    MouseInteractionType type;
    bool active;
    float worldX;
    float worldY;
    float lastWorldX;
    float lastWorldY;
    std::chrono::high_resolution_clock::time_point timestamp;
    std::chrono::high_resolution_clock::time_point lastTimestamp;
};

// ----------------------------------------------------------------------------
//  Interaction types (from simulation perspective)
// ----------------------------------------------------------------------------

/**
 * \enum InteractionType
 * \brief Type of runtime interaction effect applied in the simulation.
 */
enum class InteractionType : uint8_t {
    Repel,   ///< Repulsive interaction.
    Attract, ///< Attractive interaction.
    Empty    ///< No interaction.
};

/**
 * \struct Interaction
 * \brief Runtime interaction state with type and world-space point.
 */
struct Interaction {
    InteractionType type{InteractionType::Empty}; ///< Interaction mode.
    Vec3 point{0.0f, 0.0f, 0.0f};                 ///< Interaction world position.
};

// ----------------------------------------------------------------------------
//  Config parameter types (UI + configuration system)
// ----------------------------------------------------------------------------

/**
 * \enum ParamType
 * \brief Underlying data type of a configuration parameter.
 */
enum class ParamType {
    Number, ///< Floating-point numeric parameter.
    Binary, ///< Boolean parameter.
    String, ///< Text parameter.
    Enum    ///< Enumerated string parameter.
};

/**
 * \enum ParamRender
 * \brief Preferred rendering style for a configuration parameter in GUI.
 */
enum class ParamRender {
    Checkbox,
    Button,
    ToggleButton,
    Slider,
    Drag,
    Input
};

// ----------------------------------------------------------------------------
//  Parameter ranges
// ----------------------------------------------------------------------------

/**
 * \struct NumberRange
 * \brief Limits and step size for numeric parameters.
 */
struct NumberRange {
    float min = 0.0f;  ///< Minimum allowed value.
    float max = 1.0f;  ///< Maximum allowed value.
    float step = 0.01f;///< Increment step.
};

/**
 * \struct BinaryRange
 * \brief Flags restricting allowed boolean states.
 */
struct BinaryRange {
    bool allowTrue  = true; ///< Whether true is allowed.
    bool allowFalse = true; ///< Whether false is allowed.
};

/**
 * \struct StringRange
 * \brief Constraints for string parameters.
 */
struct StringRange {
    bool freeText = true;                 ///< Whether arbitrary text input is allowed.
    std::vector<std::string> options;     ///< Optional finite list of allowed values.
};

/**
 * \struct EnumRange
 * \brief Allowed values for enumerated parameters.
 */
struct EnumRange {
    std::vector<std::string> options; ///< Enumeration value list.
};

// ----------------------------------------------------------------------------
//  Unified parameter storage
// ----------------------------------------------------------------------------

/**
 * \typedef ParamValue
 * \brief Variant storing runtime parameter value.
 */
using ParamValue = std::variant<
    bool,
    int,
    float,
    std::string
>;

/**
 * \typedef ParamRange
 * \brief Variant storing parameter domain / constraint definition.
 */
using ParamRange = std::variant<
    NumberRange,
    BinaryRange,
    StringRange,
    EnumRange
>;
