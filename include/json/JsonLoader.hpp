#pragma once

#include <string>

#include <nlohmann/json.hpp>


namespace JsonLoader {

    nlohmann::json loadFromFile(const std::string& path);

    std::vector<std::string> listJsonFilesInDirectory(const std::string& directoryPath);
}
