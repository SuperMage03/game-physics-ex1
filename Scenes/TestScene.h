#pragma once

#include "Scene.h"
#include "FiniteDifferenceSolver.h"
#include <imgui.h>

class TestScene : public Scene {
    float f_delta = 0.01;
    bool f_pause = true;
    FiniteDifferenceSolver::FD_HPRDBC2D_Solver f_solver;
    GridFunction::ScalarGridFunction2D f_heatField;

public:
    TestScene()
    {}

    void init() override {
        f_solver = std::move(FiniteDifferenceSolver::FD_HPRDBC2D_Solver(
            HeatProblem::HeatProblemRectDBC2D(
                glm::dvec2(-2., -2.),
                glm::dvec2(4., 4.),
                1.,
                [](glm::dvec2 point) {
                    return (point.x + 2) * (point.x - 2) * (point.y + 2) * (point.y - 2) / 3.5;
                },
                [](double x) {
                    return 0.;
                },
                [](double x) {
                    return 0.;
                },
                [](double y) {
                    return 0.;
                },
                [](double y) {
                    return 0.;
                }
            )
        ));
        f_heatField = f_solver.getInitialState(3, 3);
    }

    void onDraw(Renderer &renderer) override {
        f_heatField.onDraw(renderer);
    }

    void simulateStep() override {
        if (!f_pause) {
            // f_solver.propagateStateExplicitOn(f_heatField, f_delta);
            f_solver.propagateStateImplicitOn(f_heatField, f_delta);
        }
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
        ImGui::Checkbox("Pause", &this->f_pause);
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->f_pause = !this->f_pause;
        }
    }
};

/*
void onGUI() override {
    ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1);
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