#pragma once

#include <string>

#include <nlohmann/json.hpp>


nlohmann::json loadJsonFromFile(const std::string& path);

std::vector<std::string> listJsonFilesInDirectory(const std::string& directoryPath);
