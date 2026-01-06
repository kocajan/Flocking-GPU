/**
 * \file VersionManager.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Manages versioned simulation configurations and exposes version selector parameter.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>

#include "config/Config.hpp"
#include "config/ConfigParameter.hpp"

/**
 * \class VersionManager
 * \brief Stores simulation configurations for multiple versions and exposes a selector parameter.
 *
 * Responsibilities:
 * - keeps a map of version id -> Config
 * - provides list of available versions
 * - exposes ConfigParameter "version" enum for UI selection
 * - retrieves configuration for a given version id
 */
class VersionManager {
private:
    std::unordered_map<std::string, Config> configs; ///< Stored configurations by version id.

public:
    ConfigParameter versions; ///< Enum parameter representing selectable versions.

public:

    /**
     * \brief Construct version manager from list of configurations.
     *
     * Builds internal configuration map and initializes the version selector parameter.
     *
     * \param[in] versions Versions loaded from configuration files.
     */
    explicit VersionManager(const std::vector<Config>& versions);

    /**
     * \brief Check whether a version exists.
     *
     * \param[in] version Version identifier string.
     * \return true if the version is present, false otherwise.
     */
    bool hasVersion(const std::string& version) const;

    /**
     * \brief Get list of available version identifiers.
     *
     * \return Vector of version ids.
     */
    std::vector<std::string> getAvailableVersions() const;

    /**
     * \brief Get configuration for a given version identifier.
     *
     * \param[in] version Version identifier string.
     * \return Reference to configuration associated with the version.
     */
    const Config& getSimConfig(const std::string& version) const;
};
