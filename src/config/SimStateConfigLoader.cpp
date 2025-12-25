#include "config/SimStateConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/SimStateJsonParser.hpp"

SimStateConfig loadSimStateConfig(const std::string& jsonPath) {
    const auto root = JsonLoader::loadFromFile(jsonPath);

    return parseSimStateConfig(root);
}
