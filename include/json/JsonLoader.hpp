#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace JsonLoader {

    // Loads and parses a JSON file.
    // Fails hard if file cannot be read or JSON is invalid.
    nlohmann::json loadFromFile(const std::string& path);

}
