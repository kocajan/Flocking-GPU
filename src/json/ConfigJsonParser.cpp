/**
 * \file ConfigJsonParser.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of JSON configuration object parser.
 */

#include <cassert>

#include "json/ConfigJsonParser.hpp"
#include "json/ParameterJsonParser.hpp"

Config parseConfig(const nlohmann::json& root) {
    assert(root.contains("id"));
    assert(root.contains("parameters"));
    assert(root.at("parameters").is_object());

    Config cfg;
    cfg.setConfigId(root.at("id").get<std::string>());

    const auto& groups = root.at("parameters");

    for (auto it = groups.begin(); it != groups.end(); ++it) {
        const std::string groupName = it.key();
        const auto& paramList = it.value();

        assert(paramList.is_array());

        for (const auto& p : paramList)
            cfg.add(parseParameter(p), groupName);
    }

    return cfg;
}
