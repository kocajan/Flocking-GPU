#include <fstream>
#include <cassert>

#include "json/JsonLoader.hpp"


nlohmann::json loadJsonFromFile(const std::string& path) {
    std::ifstream file(path);
    assert(file && "Failed to open JSON file");

    nlohmann::json j;
    file >> j;

    return j;
}

std::vector<std::string> listJsonFilesInDirectory(const std::string& directoryPath) {
    std::vector<std::string> filePaths;

    for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            filePaths.push_back(entry.path().string());
        }
    }

    return filePaths;
}
