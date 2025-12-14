#include "config/VersionConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/VersionJsonParser.hpp"

std::vector<VersionConfig> loadVersionConfigs(const std::string& jsonPath) {
    // Step 1: load JSON
    const auto root = JsonLoader::loadFromFile(jsonPath);

    // Step 2: parse domain-specific configs
    return parseVersionConfigs(root);
}
