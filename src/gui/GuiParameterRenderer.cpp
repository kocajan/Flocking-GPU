/**
 * \file GuiParameterRenderer.cpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of configurable parameter rendering using ImGui widgets.
 */

#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "gui/GuiParameterRenderer.hpp"

#include <imgui/imgui.h>
#include <imgui/imgui_stdlib.h>

void renderParameter(ConfigParameter& p) {

    switch (p.type) {

    // --------------------------------------------------------
    // Number
    // --------------------------------------------------------
    case ParamType::Number: {
        auto&  range = p.numberRange();
        float& value = p.number();

        switch (p.render) {

        case ParamRender::Drag: {
            const float speed = (range.step > 0.0f) ? range.step : 0.01f;

            if (ImGui::DragFloat(p.label.c_str(), &value, speed, range.min, range.max)) {
                if (range.step > 0.0f) {
                    value = range.min +
                        std::round((value - range.min) / range.step) * range.step;
                    value = std::clamp(value, range.min, range.max);
                }
            }
            break;
        }

        case ParamRender::Slider:
            ImGui::SliderFloat(p.label.c_str(), &value, range.min, range.max);
            break;

        default:
            assert(false && "Invalid ParamRender for Number");
        }
        break;
    }

    // --------------------------------------------------------
    // Binary (bool)
    // --------------------------------------------------------
    case ParamType::Binary: {
        bool& value = p.binary();

        switch (p.render) {

        case ParamRender::Checkbox:
            ImGui::Checkbox(p.label.c_str(), &value);
            break;

        case ParamRender::Button:
            ImGui::Button(p.label.c_str());
            value = ImGui::IsItemActive();
            break;

        case ParamRender::ToggleButton: {
            if (value) {
                ImGui::PushStyleColor(
                    ImGuiCol_Button,
                    ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive)
                );
            }

            if (ImGui::Button(p.label.c_str()))
                value = !value;

            if (value)
                ImGui::PopStyleColor();
            break;
        }

        default:
            assert(false && "Invalid ParamRender for Binary");
        }
        break;
    }

    // --------------------------------------------------------
    // String
    // --------------------------------------------------------
    case ParamType::String: {

        switch (p.render) {

        case ParamRender::Input:
            ImGui::InputText(p.label.c_str(), &p.string());
            break;

        default:
            assert(false && "Invalid ParamRender for String");
        }
        break;
    }

    // --------------------------------------------------------
    // Enum
    // --------------------------------------------------------
    case ParamType::Enum: {
        auto&        options = p.enumOptions();
        std::string& current = p.string();

        assert(
            p.render == ParamRender::Input &&
            "Enum currently supports only Combo-style input"
        );

        int index = 0;
        for (int i = 0; i < static_cast<int>(options.size()); ++i)
            if (options[i] == current)
                index = i;

        std::vector<const char*> labels;
        labels.reserve(options.size());

        for (auto& s : options)
            labels.push_back(s.c_str());

        if (ImGui::Combo(
                p.label.c_str(),
                &index,
                labels.data(),
                static_cast<int>(labels.size())
            )) {
            current = options[index];
        }
        break;
    }

    // --------------------------------------------------------
    // Unknown / unsupported
    // --------------------------------------------------------
    default:
        throw std::runtime_error("Unknown ParamType in renderParameter");
    }
}
