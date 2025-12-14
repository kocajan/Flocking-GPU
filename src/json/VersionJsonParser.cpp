#include "json/VersionJsonParser.hpp"
#include "json/ParameterParser.hpp"

#include <cassert>

std::vector<VersionConfig> parseVersionConfigs(const nlohmann::json& root) {
    std::vector<VersionConfig> result;

    for (const auto& v : root.at("versions")) {
        VersionConfig vc;
        vc.versionId = v.at("id").get<std::string>();

        for (const auto& p : v.at("parameters")) {
            vc.parameters.push_back(parseParameter(p));
        }

        result.push_back(std::move(vc));
    }

    return result;
}
