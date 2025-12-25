#include "config/VersionConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/ConfigJsonParser.hpp"


std::vector<Config> loadVersionConfigs(const std::string& jsonPath) {
    const auto filePaths = JsonLoader::listJsonFilesInDirectory(jsonPath);

    std::vector<Config> versionConfigs;
    for (const auto& path : filePaths) {
        const auto root = JsonLoader::loadFromFile(path);
        versionConfigs.push_back(parseConfig(root));
    }
    return versionConfigs;
}
