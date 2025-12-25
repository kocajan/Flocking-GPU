#include "config/VersionConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/VersionJsonParser.hpp"

std::vector<VersionConfig> loadVersionConfigs(const std::string& jsonPath) {
    const auto root = JsonLoader::loadFromFile(jsonPath);

    return parseVersionConfigs(root);
}
