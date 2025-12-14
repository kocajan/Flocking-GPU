#include "json/VersionJsonParser.hpp"

#include <cassert>

static ConfigParameter parseParameter(const nlohmann::json& j) {
    const std::string type = j.at("type").get<std::string>();
    const std::string name = j.at("name").get<std::string>();
    const std::string label = j.value("label", name);
    const std::string desc  = j.value("description", "");

    if (type == "number") {
        return ConfigParameter::Number(
            name, label, desc,
            j.at("default").get<float>(),
            j.at("min").get<float>(),
            j.at("max").get<float>(),
            j.value("step", 0.01f)
        );
    }

    if (type == "binary") {
        return ConfigParameter::Binary(
            name, label, desc,
            j.at("default").get<bool>()
        );
    }

    if (type == "string") {
        return ConfigParameter::String(
            name, label, desc,
            j.at("default").get<std::string>(),
            j.value("freeText", true),
            j.value("options", std::vector<std::string>{})
        );
    }

    if (type == "enum") {
        return ConfigParameter::Enum(
            name, label, desc,
            j.at("default").get<std::string>(),
            j.at("options").get<std::vector<std::string>>()
        );
    }

    assert(false && "Unknown parameter type");
    return {};
}

std::vector<VersionConfig>
parseVersionConfigs(const nlohmann::json& root) {
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
