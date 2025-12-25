#include "config/ConfigLoader.hpp"

#include "json/JsonLoader.hpp"
#include "json/ConfigJsonParser.hpp"


Config loadConfig(const std::string& jsonPath) {
    const auto root = loadJsonFromFile(jsonPath);

    return parseConfig(root);
}
