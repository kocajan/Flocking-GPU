#include "json/JsonLoader.hpp"

#include <fstream>
#include <cassert>

namespace JsonLoader {

nlohmann::json loadFromFile(const std::string& path) {
    std::ifstream file(path);
    assert(file && "Failed to open JSON file");

    nlohmann::json j;
    file >> j;   // throws on parse error

    return j;
}

}
