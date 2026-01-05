/**
 * \file ParameterJsonParser.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Parsing of individual configuration parameters from JSON objects.
 */

#pragma once

#include <vector>
#include <nlohmann/json.hpp>

#include "config/ConfigParameter.hpp"

/**
 * \brief Parse a configuration parameter from JSON.
 *
 * Supported parameter kinds:
 * - number
 * - binary
 * - string
 * - enum
 *
 * The JSON object is expected to contain fields such as:
 * - "type"
 * - "name"
 * - "default"
 * - optional metadata (label, description, options, etc.)
 *
 * \param[in] j JSON node describing a parameter.
 * \return Constructed ConfigParameter instance.
 */
ConfigParameter parseParameter(const nlohmann::json& j);
