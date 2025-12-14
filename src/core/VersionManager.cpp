#include "core/VersionManager.hpp"

VersionManager::VersionManager(const std::vector<VersionConfig>& versions) {
    for (const auto& v : versions) {
        assert(configs.find(v.versionId) == configs.end());

        SimConfig cfg;
        for (const auto& param : v.parameters) {
            cfg.add(param); // copy parameter template
        }

        configs.emplace(v.versionId, std::move(cfg));
    }
}

bool VersionManager::hasVersion(const VersionId& version) const {
    return configs.find(version) != configs.end();
}

std::vector<VersionManager::VersionId> VersionManager::getAvailableVersions() const {
    std::vector<VersionId> versions;
    versions.reserve(configs.size());
    for (const auto& pair : configs) {
        versions.push_back(pair.first);
    }
    return versions;
}

const SimConfig& VersionManager::getSimConfig(const VersionId& version) const {
    auto it = configs.find(version);
    assert(it != configs.end());
    return it->second;
}
