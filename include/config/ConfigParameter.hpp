#pragma once

#include <string>
#include <variant>
#include <vector>
#include <cassert>

#include "core/Types.hpp"


class ConfigParameter {
public:
    // Identity / metadata
    std::string name;            // stable identifier (e.g. "time_scale")
    std::string label;           // UI label (e.g. "Time scale")
    std::string description;     // documentation / tooltip
    
    // Type
    ParamType type;

    // Value
    ParamValue value;
    ParamValue defaultValue;

    // Range / domain
    ParamRange range;

    // Render
    ParamRender render;

public:
    // Factory constructors
    static ConfigParameter Number(
        std::string name,
        std::string label,
        std::string description,
        float defaultValue,
        float min,
        float max,
        float step,
        ParamRender render = ParamRender::Drag
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Number,
            defaultValue,
            defaultValue,
            NumberRange{min, max, step},
            render
        };
    }

    static ConfigParameter Binary(
        std::string name,
        std::string label,
        std::string description,
        bool defaultValue,
        ParamRender render = ParamRender::Checkbox
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Binary,
            defaultValue,
            defaultValue,
            BinaryRange{},
            render
        };
    }

    static ConfigParameter String(
        std::string name,
        std::string label,
        std::string description,
        std::string defaultValue,
        bool freeText = true,
        std::vector<std::string> options = {},
        ParamRender render = ParamRender::Input
    ) {
        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::String,
            defaultValue,
            defaultValue,
            StringRange{freeText, std::move(options)},
            render
        };
    }

    static ConfigParameter Enum(
        std::string name,
        std::string label,
        std::string description,
        std::string defaultValue,
        std::vector<std::string> options,
        ParamRender render = ParamRender::Input
    ) {
        assert(!options.empty());

        return {
            std::move(name),
            std::move(label),
            std::move(description),
            ParamType::Enum,
            defaultValue,
            defaultValue,
            EnumRange{std::move(options)},
            render
        };
    }

    // Define typed accessors for value
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

    // ----- Range accessors -----

    NumberRange& numberRange() {
        assert(type == ParamType::Number);
        return std::get<NumberRange>(range);
    }

    const NumberRange& numberRange() const {
        assert(type == ParamType::Number);
        return std::get<NumberRange>(range);
    }

    BinaryRange& binaryRange() {
        assert(type == ParamType::Binary);
        return std::get<BinaryRange>(range);
    }

    const BinaryRange& binaryRange() const {
        assert(type == ParamType::Binary);
        return std::get<BinaryRange>(range);
    }

    StringRange& stringRange() {
        assert(type == ParamType::String);
        return std::get<StringRange>(range);
    }

    const StringRange& stringRange() const {
        assert(type == ParamType::String);
        return std::get<StringRange>(range);
    }

    EnumRange& enumRange() {
        assert(type == ParamType::Enum);
        return std::get<EnumRange>(range);
    }

    const EnumRange& enumRange() const {
        assert(type == ParamType::Enum);
        return std::get<EnumRange>(range);
    }

    // ----- Convenience helpers -----

    // Enum option list
    std::vector<std::string>& enumOptions() {
        return enumRange().options;
    }

    const std::vector<std::string>& enumOptions() const {
        return enumRange().options;
    }

    // String allowed options (if not free-text)
    std::vector<std::string>& stringOptions() {
        return stringRange().options;
    }

    const std::vector<std::string>& stringOptions() const {
        return stringRange().options;
    }

    // Reset to default value
    void reset() {
        value = defaultValue;
    }
};
