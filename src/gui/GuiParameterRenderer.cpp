#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "gui/GuiParameterRenderer.hpp"

#include <imgui/imgui.h>
#include <imgui/imgui_stdlib.h>


void renderParameter(ConfigParameter& p) {
    switch (p.type) {

    case ParamType::Number: {
        auto& r = std::get<NumberRange>(p.range);
        float& v = p.number();

        switch (p.render) {
        case ParamRender::Drag: {
            const float speed = (r.step > 0.0f) ? r.step : 0.01f;
            if (ImGui::DragFloat(p.label.c_str(), &v, speed, r.min, r.max)) {
                if (r.step > 0.0f) {
                    v = r.min + std::round((v - r.min) / r.step) * r.step;
                    v = std::clamp(v, r.min, r.max);
                }
            }
            break;
        }

        case ParamRender::Slider:
            ImGui::SliderFloat(p.label.c_str(), &v, r.min, r.max);
            break;

        default:
            assert(false && "Invalid ParamRender for Number");
        }
        break;
    }

    case ParamType::Binary: {
        bool& v = p.binary();

        switch (p.render) {
        case ParamRender::Checkbox:
            ImGui::Checkbox(p.label.c_str(), &v);
            break;

        case ParamRender::Button:
            ImGui::Button(p.label.c_str());
            v = ImGui::IsItemActive();   // pressed = true, released = false
            break;

        case ParamRender::ToggleButton:
            if (v)
                ImGui::PushStyleColor(
                    ImGuiCol_Button,
                    ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive)
                );

            if (ImGui::Button(p.label.c_str()))
                v = !v;

            if (v)
                ImGui::PopStyleColor();
            break;

        default:
            assert(false && "Invalid ParamRender for Binary");
        }
        break;
    }

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

    case ParamType::Enum: {
        auto& r = std::get<EnumRange>(p.range);
        std::string& cur = p.string();

        assert(p.render == ParamRender::Input &&
               "Enum currently supports only Combo-style input");

        int idx = 0;
        for (int i = 0; i < (int)r.options.size(); ++i)
            if (r.options[i] == cur)
                idx = i;

        std::vector<const char*> labels;
        labels.reserve(r.options.size());
        for (auto& s : r.options)
            labels.push_back(s.c_str());

        if (ImGui::Combo(
                p.label.c_str(),
                &idx,
                labels.data(),
                (int)labels.size()
            )) {
            cur = r.options[idx];
        }
        break;
    }
    default:
        throw std::runtime_error("Unknown ParamType in renderParameter");
    }
}
