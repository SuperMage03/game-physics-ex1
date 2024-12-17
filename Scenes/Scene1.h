#pragma once

#include "Scene.h"
#include <imgui.h>

class Scene1 : public Scene {
    float f_delta = 0.01;
    bool f_pause = true;
    bool f_singleStep = false;

    glm::dmat4 f_cameraMatrix = glm::dmat4(1);
    glm::dvec3 f_fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 f_right = glm::dvec3(0, 1, 0);
    glm::dvec3 f_up = glm::dvec3(0, 0, 1);

public:
    void init() override {

    }

    void onDraw(Renderer &renderer) override {
        f_cameraMatrix = renderer.camera.viewMatrix;
        f_fwd = inverse(f_cameraMatrix) * glm::dvec4(0, 0, 1, 0);
        f_right = inverse(f_cameraMatrix) * glm::dvec4(1, 0, 0, 0);
        f_up = inverse(f_cameraMatrix) * glm::dvec4(0, 1, 0, 0);
    }

    void simulateStep() override {
        if (!f_pause) {

        }
        else {
            if (f_singleStep) {
                
                f_singleStep = false;
            }
        }
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
        ImGui::Text("Space : pause/unpause");
        ImGui::Checkbox("Pause", &this->f_pause);
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->f_pause = !this->f_pause;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_S)) {
            this->f_singleStep = true;
        }
    }
};

/*
void onGUI() override {
    ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
    ImGui::Checkbox("Pause", &this->f_pause);
    ImGui::Checkbox("Gravity", &this->gravity);
    ImGui::Text("RMB + drag : apply force to one of the objects.");
    ImGui::Text("Space : f_pause/unpause");
    if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
        this->f_pause = !this->f_pause;
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
}*/