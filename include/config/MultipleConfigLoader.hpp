/**
 * \file MultipleConfigLoader.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Utilities for loading multiple configuration files.
 */

#pragma once

#include <vector>
#include <string>

#include "config/Config.hpp"

/**
 * \brief Load all configuration files from a directory.
 *
 * Scans the directory for JSON files and parses each into a Config instance.
 *
 * \param[in] jsonPath Path to directory containing version configuration files.
 * \return Vector of loaded configuration objects.
 */
std::vector<Config> loadConfigs(const std::string& jsonPath);
