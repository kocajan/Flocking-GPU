#pragma once

#include <string>
#include <variant>
#include <vector>
#include <cassert>

// ============================================================
// High-level parameter type (semantic, not primitive)
// ============================================================

enum class ParamType {
    Number,     // numeric value with range and step size
    Binary,     // true / false
    String,     // string (free or constrained)
    Enum,       // one of predefined options
    Custom      // user-defined type
};

// ============================================================
// Value storage
// ============================================================

using ParamValue = std::variant<
    bool,
    int,
    float,
    std::string
>;

// ============================================================
// Range / domain definitions
// ============================================================

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
    std::vector<std::string> options;          // must be non-empty
};

// ---------- Custom ----------
struct CustomRange {
    void* userData = nullptr;                  // opaque payload
};

// ============================================================
// Unified range container
// ============================================================

using ParamRange = std::variant<
    NumberRange,
    BinaryRange,
    StringRange,
    EnumRange,
    CustomRange
>;

// ============================================================
// Configuration parameter class
// ============================================================

class ConfigParameter {
public:
    // --------------------------------------------------------
    // Identity / metadata (ALWAYS PRESENT)
    // --------------------------------------------------------
    std::string name;            // stable identifier (e.g. "time_scale")
    std::string label;           // UI label (e.g. "Time scale")
    std::string description;     // documentation / tooltip
    ParamType type;

    // --------------------------------------------------------
    // Value
    // --------------------------------------------------------
    ParamValue value;
    ParamValue defaultValue;

    // --------------------------------------------------------
    // Range / domain
    // --------------------------------------------------------
    ParamRange range;

public:
    // ========================================================
    // Factory constructors
    // ========================================================

    static ConfigParameter Number(
        std::string name,
        std::string label,
        std::string description,
        float defaultValue,
        float min,
        float max,
        float step
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Number,
            defaultValue,
            defaultValue,
            NumberRange{min, max, step}
        };
    }

    static ConfigParameter Binary(
        std::string name,
        std::string label,
        std::string description,
        bool defaultValue
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Binary,
            defaultValue,
            defaultValue,
            BinaryRange{}
        };
    }

    static ConfigParameter String(
        std::string name,
        std::string label,
        std::string description,
        std::string defaultValue,
        bool freeText = true,
        std::vector<std::string> options = {}
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::String,
            defaultValue,
            defaultValue,
            StringRange{freeText, std::move(options)}
        };
    }

    static ConfigParameter Enum(
        std::string name,
        std::string label,
        std::string description,
        std::string defaultValue,
        std::vector<std::string> options
    ) {
        assert(!options.empty());

        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Enum,
            defaultValue,
            defaultValue,
            EnumRange{std::move(options)}
        };
    }

    static ConfigParameter Custom(
        std::string name,
        std::string label,
        std::string description,
        ParamValue defaultValue,
        CustomRange range
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Custom,
            defaultValue,
            defaultValue,
            range
        };
    }

    // ========================================================
    // Typed accessors
    // ========================================================

    float& number() {
        assert(type == ParamType::Number);
        return std::get<float>(value);
    }

    bool& binary() {
        assert(type == ParamType::Binary);
        return std::get<bool>(value);
    }

    std::string& string() {
        assert(type == ParamType::String || type == ParamType::Enum);
        return std::get<std::string>(value);
    }

    const float& number() const {
        assert(type == ParamType::Number);
        return std::get<float>(value);
    }

    const bool& binary() const {
        assert(type == ParamType::Binary);
        return std::get<bool>(value);
    }

    const std::string& string() const {
        assert(type == ParamType::String || type == ParamType::Enum);
        return std::get<std::string>(value);
    }

    // ========================================================
    // Utilities
    // ========================================================

    void reset() {
        value = defaultValue;
    }
};
