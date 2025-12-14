#include "config/SimStateConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/SimStateJsonParser.hpp"

SimStateConfig loadSimStateConfig(const std::string& jsonPath) {
    // Step 1: load JSON
    const auto root = JsonLoader::loadFromFile(jsonPath);

    // Step 2: parse domain-specific config
    return parseSimStateConfig(root);
}
