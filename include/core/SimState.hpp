/**
 * \file SimState.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Simulation runtime state and parameter bindings.
 *
 * Holds:
 * - configuration-derived parameters
 * - runtime control values
 * - world geometry and rendering attributes
 * - population and interaction state
 * - SoA boid storage
 *
 * Provides reset helpers for restoring initial or reloaded configuration.
 */

#pragma once

#include <string>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "config/ConfigParameter.hpp"

/**
 * \class SimState
 * \brief Aggregates simulation state, runtime control fields, and parameter bindings.
 *
 * Responsibilities:
 * - copy configuration parameters into runtime-accessible fields
 * - expose reset utilities (to defaults or new configuration)
 * - provide lookup helpers for parameters
 * - own SoA container storing boid population
 */
class SimState {
public:
    // Persistent initial configuration (used for reset-to-defaults)
    Config initialConfig;        ///< Stored baseline configuration.
    ConfigParameter initialVersionParam; ///< Initial version selector parameter.

    // Simulation control
    ConfigParameter dt;              ///< Simulation timestep.
    ConfigParameter paused;          ///< Pause toggle flag.
    uint64_t tick = 0;               ///< Simulation step counter.
    ConfigParameter dimensions;      ///< Dimensionality of world (2D/3D).

    // Population targets
    ConfigParameter basicBoidCountTarget;
    ConfigParameter predatorBoidCountTarget;

    // Interaction & input effects
    ConfigParameter leftMouseEffect;
    ConfigParameter rightMouseEffect;
    Interaction interaction;

    // Reset / control commands
    ConfigParameter resetVersionSettings;
    ConfigParameter resetSimulation;
    ConfigParameter deleteObstacles;

    // World geometry
    ConfigParameter worldX;
    ConfigParameter worldY;
    ConfigParameter worldZ;

    // Rendering colors
    ConfigParameter basicBoidColor;
    ConfigParameter predatorBoidColor;
    ConfigParameter obstacleBoidColor;

    // Radii / physical footprint
    ConfigParameter obstacleBoidRadius;
    ConfigParameter predatorBoidRadius;
    ConfigParameter basicBoidRadius;

    // Population dynamics / constraints
    ConfigParameter maxBoidPopulationChangeRate;

    // Initial random velocity range
    ConfigParameter initialAxialSpeedRange;

    // Numerical stability parameters
    ConfigParameter eps;

    // Version / implementation selector
    ConfigParameter version;

    // Boid population storage (SoA layout)
    Boids boids;

    /**
     * \brief Reset runtime state using a new configuration.
     *
     * Stores the given configuration as the new baseline and rebinds all
     * runtime parameters to match it.
     *
     * \param[in] config New configuration to load.
     * \param[in] versionParam Version selector parameter associated with config.
     */
    void resetToNewConfig(const Config& config, ConfigParameter versionParam);

    /**
     * \brief Reset runtime state to the configuration designated as initial now.
     */
    void resetToDefaults();

    /**
     * \brief Check whether a parameter exists in the initial configuration.
     *
     * \param[in] name Parameter name.
     * \return true if the parameter exists, false otherwise.
     */
    bool has(const std::string& name) const;

    /**
     * \brief Get parameter by name from the initial configuration.
     *
     * \param[in] name Parameter name.
     * \return Reference to configuration parameter.
     */
    ConfigParameter& get(const std::string& name);

    /**
     * \brief Get parameter by name (const overload).
     *
     * \param[in] name Parameter name.
     * \return Const reference to configuration parameter.
     */
    const ConfigParameter& get(const std::string& name) const;

    /**
     * \brief Construct runtime state from a configuration.
     *
     * \param[in] config Configuration source.
     * \param[in] versionParam Version selector parameter.
     */
    SimState(const Config& config, ConfigParameter versionParam);
};
