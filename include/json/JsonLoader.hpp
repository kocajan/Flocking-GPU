/**
 * \file JsonLoader.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief JSON file loading utilities for configuration files.
 */

#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

/**
 * \brief Load JSON data from a file.
 *
 * \param[in] path Path to JSON file on disk.
 * \return Parsed JSON object.
 */
nlohmann::json loadJsonFromFile(const std::string& path);

/**
 * \brief List all JSON files in a directory.
 *
 * \param[in] directoryPath Path to directory to scan.
 * \return Vector of file paths to .json files.
 */
std::vector<std::string> listJsonFilesInDirectory(const std::string& directoryPath);
