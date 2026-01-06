/**
 * \file ConfigLoader.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Utilities for loading configuration from JSON files.
 */

#pragma once

#include <string>

#include "config/Config.hpp"

/**
 * \brief Load configuration from a JSON file.
 *
 * Parses a JSON document and constructs a Config instance.
 *
 * \param[in] jsonPath Path to JSON configuration file.
 * \return Loaded configuration object.
 */
Config loadConfig(const std::string& jsonPath);
