/**
 * \file ConfigJsonParser.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Parsing of configuration objects from JSON structures.
 */

#pragma once

#include <vector>

#include "nlohmann/json.hpp"
#include "config/Config.hpp"

/**
 * \brief Parse a configuration from a JSON object.
 *
 * Expects the following structure:
 * - "id"           : string configuration identifier
 * - "parameters"   : object of parameter groups, each containing an array
 *
 * \param[in] root JSON node representing a configuration object.
 * \return Constructed Config instance.
 */
Config parseConfig(const nlohmann::json& root);
