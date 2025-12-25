#include "core/VersionManager.hpp"

VersionManager::VersionManager(const std::vector<Config>& versionsInput) {
    assert(!versionsInput.empty());

    std::vector<std::string> versionIds;
    versionIds.reserve(versionsInput.size());

    for (const auto& v : versionsInput) {
        assert(configs.find(v.getConfigId()) == configs.end());

        // Build Config for this version
        Config cfg;
        for (const auto& param : v.getParameters()) {
            cfg.add(param); // copy parameter template
        }

        configs.emplace(v.getConfigId(), std::move(cfg));
        versionIds.push_back(v.getConfigId());
    }

    // Create enum parameter for version selection
    versions = ConfigParameter::Enum(
        "version",
        "Version",
        "Simulation version",
        versionIds.front(),   // default = first version
        versionIds            // options
    );
}

bool VersionManager::hasVersion(const std::string& version) const {
    return configs.find(version) != configs.end();
}

std::vector<std::string> VersionManager::getAvailableVersions() const {
    std::vector<std::string> versions;
    versions.reserve(configs.size());
    for (const auto& pair : configs) {
        versions.push_back(pair.first);
    }
    return versions;
}

const Config& VersionManager::getSimConfig(const std::string& version) const {
    auto it = configs.find(version);
    assert(it != configs.end());
    return it->second;
}
