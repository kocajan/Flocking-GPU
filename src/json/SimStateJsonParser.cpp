#include "json/SimStateJsonParser.hpp"
#include "json/ParameterParser.hpp"

#include <cassert>

SimStateConfig parseSimStateConfig(const nlohmann::json& root) {
    SimStateConfig sc;

    assert(root.contains("parameters"));
    assert(root.at("parameters").is_array());

    for (const auto& p : root.at("parameters")) {
        sc.add(parseParameter(p));
    }

    return sc;
}
