/**
 * \file ConfigParameter.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Strongly-typed configuration parameter with metadata, range constraints,
 *        default value, and UI rendering hints.
 */

#pragma once

#include <string>
#include <variant>
#include <vector>
#include <cassert>

#include "core/Types.hpp"

/**
 * \class ConfigParameter
 * \brief Configuration parameter with value, default value, type, range, and UI metadata.
 */
class ConfigParameter {
public:
    // Identity / metadata
    std::string name;        ///< Stable identifier (e.g. "time_scale").
    std::string label;       ///< UI label (e.g. "Time scale").
    std::string description; ///< Documentation / tooltip.

    // Type
    ParamType type;          ///< Parameter value type.

    // Value
    ParamValue value;        ///< Current value.
    ParamValue defaultValue; ///< Default value used for reset.

    // Range / domain
    ParamRange range;        ///< Allowed value domain / constraints.

    // Render hint
    ParamRender render;      ///< Preferred UI rendering mode.

public:
    // -------------------------------------------------------------------------
    // Factory constructors
    // -------------------------------------------------------------------------

    /**
     * \brief Create a numeric parameter.
     */
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

    /**
     * \brief Create a binary (boolean) parameter.
     */
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

    /**
     * \brief Create a string parameter (free-text or restricted options).
     */
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

    /**
     * \brief Create an enum parameter.
     *
     * The value must be one of the provided options.
     */
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

    // -------------------------------------------------------------------------
    // Typed value accessors
    // -------------------------------------------------------------------------

    /**
     * \brief Access numeric value (non-const).
     */
    float& number() {
        assert(type == ParamType::Number);
        return std::get<float>(value);
    }

    /**
     * \brief Access binary value (non-const).
     */
    bool& binary() {
        assert(type == ParamType::Binary);
        return std::get<bool>(value);
    }

    /**
     * \brief Access string/enum value (non-const).
     */
    std::string& string() {
        assert(type == ParamType::String || type == ParamType::Enum);
        return std::get<std::string>(value);
    }

    /**
     * \brief Access numeric value (const).
     */
    const float& number() const {
        assert(type == ParamType::Number);
        return std::get<float>(value);
    }

    /**
     * \brief Access binary value (const).
     */
    const bool& binary() const {
        assert(type == ParamType::Binary);
        return std::get<bool>(value);
    }

    /**
     * \brief Access string/enum value (const).
     */
    const std::string& string() const {
        assert(type == ParamType::String || type == ParamType::Enum);
        return std::get<std::string>(value);
    }

    // -------------------------------------------------------------------------
    // Range accessors
    // -------------------------------------------------------------------------

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

    // -------------------------------------------------------------------------
    // Convenience helpers
    // -------------------------------------------------------------------------

    /**
     * \brief Get modifiable list of enum options.
     */
    std::vector<std::string>& enumOptions() {
        return enumRange().options;
    }

    /**
     * \brief Get read-only list of enum options.
     */
    const std::vector<std::string>& enumOptions() const {
        return enumRange().options;
    }

    /**
     * \brief Get modifiable list of allowed string options (if not free-text).
     */
    std::vector<std::string>& stringOptions() {
        return stringRange().options;
    }

    /**
     * \brief Get read-only list of allowed string options (if not free-text).
     */
    const std::vector<std::string>& stringOptions() const {
        return stringRange().options;
    }

    /**
     * \brief Reset parameter value back to its default value.
     */
    void reset() {
        value = defaultValue;
    }
};
