/**
 * \file Config.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Base configuration container storing typed parameters grouped by category.
 *
 * Provides lookup, access helpers, grouping metadata, and reset-to-defaults support.
 */

#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include "config/ConfigParameter.hpp"

/**
 * \class Config
 * \brief Generic configuration parameter set with grouping metadata.
 *
 * Responsibilities:
 * - stores parameters and provides fast lookup by name
 * - groups parameters into named categories (with ordering)
 * - exposes typed getters and reference accessors
 * - allows resetting all parameters to default values
 */
class Config {
public:

    /**
     * \brief Construct configuration with optional identifier.
     *
     * \param[in] id Configuration identifier string (optional).
     */
    explicit Config(std::string id = {}) : configId(std::move(id)) {}

    virtual ~Config();

    // --- Configuration identifier ---

    /**
     * \brief Get configuration identifier.
     *
     * \return Reference to configuration id string.
     */
    const std::string& getConfigId() const;

    /**
     * \brief Set configuration identifier.
     *
     * \param[in] id New configuration id value.
     */
    void setConfigId(std::string id);

    // --- Parameter lookup and access API ---

    /**
     * \brief Add a configuration parameter and assign it to a group.
     *
     * \param[in] param Parameter definition.
     * \param[in] groupName Name of group to assign parameter to.
     */
    void add(ConfigParameter param, const std::string& groupName);

    /**
     * \brief Check whether a parameter exists.
     *
     * \param[in] name Parameter name.
     * \return true if parameter exists, false otherwise.
     */
    bool has(const std::string& name) const;

    /**
     * \brief Get parameter by name.
     *
     * \param[in] name Parameter name.
     * \return Reference to parameter.
     */
    ConfigParameter& get(const std::string& name);

    /**
     * \brief Get parameter by name (const overload).
     *
     * \param[in] name Parameter name.
     * \return Const reference to parameter.
     */
    const ConfigParameter& get(const std::string& name) const;

    /**
     * \brief Get mutable list of all parameters.
     *
     * \return Vector of parameters.
     */
    std::vector<ConfigParameter>& getParameters();

    /**
     * \brief Get read-only list of all parameters.
     *
     * \return Const vector of parameters.
     */
    const std::vector<ConfigParameter>& getParameters() const;

    // --- Typed value helpers ---

    float number(const std::string& name) const;
    bool binary(const std::string& name) const;
    const std::string& string(const std::string& name) const;

    float& numberRef(const std::string& name);
    bool& binaryRef(const std::string& name);
    std::string& stringRef(const std::string& name);

    /**
     * \brief Reset all parameters to their default values.
     */
    virtual void resetAll();

    // --- Grouping metadata accessors ---

    /**
     * \brief Map of parameter name -> group name.
     *
     * \return Association table of parameter to group.
     */
    const std::unordered_map<std::string, std::string>& getParamGroups() const {
        return paramGroupByName;
    }

    /**
     * \brief Map of group name -> parameter names.
     *
     * \return Association between groups and contained parameters.
     */
    const std::unordered_map<std::string, std::vector<std::string>>& getGroupParams() const {
        return groupParams;
    }

    /**
     * \brief Ordered list of group names.
     *
     * \return Group ordering as defined during insertion.
     */
    const std::vector<std::string>& getGroupOrder() const {
        return groupOrder;
    }

private:
    std::string configId;
    std::vector<ConfigParameter> params;
    std::unordered_map<std::string, std::size_t> indexByName;

    // Grouping structures
    std::unordered_map<std::string, std::string> paramGroupByName;
    std::unordered_map<std::string, std::vector<std::string>> groupParams;
    std::vector<std::string> groupOrder;
};
