#include "Scene.h"

class Scene1 : public Scene
{
    /*
    void init() override {

    }

    void onDraw(Renderer &renderer) override {
        
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1);
        ImGui::Checkbox("Pause", &this->pause);
        ImGui::Checkbox("Gravity", &this->gravity);
        ImGui::Text("RMB + drag : apply force to one of the objects.");
        ImGui::Text("Space : pause/unpause");
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->pause = !this->pause;
        }
        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){   
            auto drag = ImGui::GetMouseDragDelta(1);
            if(drag.x != 0 || drag.y != 0) {
                glm::dvec3 dx = (double)(drag.x) * right;
                glm::dvec3 dy = (double)(-drag.y) * up;
                ROPE.applyForceToObject(
                    0,
                    Force (
                        (dx + dy) / 25.,
                        ROPE.getObject(0).f_transform.f_position
                    )
                );
            }
        }
    }

*/
};