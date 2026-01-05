/**
 * \file ConfigLoader.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of JSON configuration loading helpers.
 */

#include "config/ConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/ConfigJsonParser.hpp"


Config loadConfig(const std::string& jsonPath) {
    const auto root = loadJsonFromFile(jsonPath);
    return parseConfig(root);
}
