#include <cassert>

#include "json/ConfigJsonParser.hpp"
#include "json/ParameterJsonParser.hpp"


Config parseConfig(const nlohmann::json& root) {
    // Validate root structure
    assert(root.contains("id"));
    assert(root.contains("parameters"));
    assert(root.at("parameters").is_array());
    
    // Create config
    Config sc;

    sc.setConfigId(root.at("id").get<std::string>());
    for (const auto& p : root.at("parameters")) {
        sc.add(parseParameter(p));
    }

    return sc;
}
