/**
 * \file MultipleConfigLoader.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of multi-file version configuration loader.
 */

#include "config/MultipleConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/ConfigJsonParser.hpp"


std::vector<Config> loadConfigs(const std::string& jsonPath) {
    const auto filePaths = listJsonFilesInDirectory(jsonPath);

    std::vector<Config> versionConfigs;
    for (const auto& path : filePaths) {
        const auto root = loadJsonFromFile(path);
        versionConfigs.push_back(parseConfig(root));
    }
    return versionConfigs;
}
