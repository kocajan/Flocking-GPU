#include <cmath>
#include <algorithm>

#include "gui/GuiParameterRenderer.hpp"

#include <imgui/imgui.h>
#include <imgui/imgui_stdlib.h>

void renderParameter(ConfigParameter& p) {
    switch (p.type) {
    case ParamType::Number: {
        auto& r = std::get<NumberRange>(p.range);

        float& v = p.number();

        const float speed = (r.step > 0.0f) ? r.step : 0.01f;

        if (ImGui::DragFloat(p.label.c_str(), &v, speed, r.min, r.max)) {
            if (r.step > 0.0f) {
                v = r.min + std::round((v - r.min) / r.step) * r.step;
                v = std::clamp(v, r.min, r.max);
            }
        }
        break;
    }
    
    case ParamType::Binary:
        ImGui::Checkbox(
            p.label.c_str(),
            &p.binary()
        );
        break;

    case ParamType::String:
        ImGui::InputText(
            p.label.c_str(),
            &p.string()
        );
        break;

    case ParamType::Enum: {
        auto& r = std::get<EnumRange>(p.range);
        std::string& cur = p.string();

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

    case ParamType::Custom:
        break;
    }
}
