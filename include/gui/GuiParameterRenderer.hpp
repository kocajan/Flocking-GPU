/**
 * \file GuiParameterRenderer.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Rendering of configuration parameters using ImGui widgets.
 */

#pragma once

#include "config/ConfigParameter.hpp"

/**
 * \brief Render a configuration parameter using its preferred GUI widget.
 *
 * The widget type depends on:
 * - parameter type (number / binary / string / enum)
 * - selected ParamRender mode
 *
 * \param[in,out] p Parameter to render and modify.
 */
void renderParameter(ConfigParameter& p);
