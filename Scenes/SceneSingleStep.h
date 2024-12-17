#pragma once

#include "Scene.h"
#include "FiniteDifferenceSolver.h"
#include <imgui.h>

class SceneSingleStep : public Scene {
    float f_delta = 0.1;
    unsigned f_n = 5;
    unsigned f_m = 8;
    glm::dvec3 f_shift = glm::dvec3(0.);
    FiniteDifferenceSolver::FD_HPRDBC2D_Solver f_solver;
    GridFunction::ScalarGridFunction2D f_heatField;
    GridFunction::ScalarGridFunction2D f_heatFieldCopy;

    glm::dmat4 f_cameraMatrix = glm::dmat4(1);
    glm::dvec3 f_fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 f_right = glm::dvec3(0, 1, 0);
    glm::dvec3 f_up = glm::dvec3(0, 0, 1);

public:
    void init() override {
        f_solver = std::move(FiniteDifferenceSolver::FD_HPRDBC2D_Solver(
            HeatProblem::HeatProblemRectDBC2D(
                glm::dvec2(0., 0.),
                glm::dvec2(2., 4.),
                0.1,
                [](glm::dvec2 point) {
                    return 0.;
                },
                [](glm::dvec2 point) {
                    return 0.;
                },
                [](double y) {
                    return 0.;
                },
                [](double y) {
                    return 0.;
                },
                [](double x) {
                    return 0.;
                },
                [](double x) {
                    return 0.;
                }
            )
        ));

        f_heatField = f_solver.getInitialState(f_n, f_m);
        
        f_heatField.setValue(1, 1, 6);
        f_heatField.setValue(1, 2, 5);
        f_heatField.setValue(1, 3, 1);
        f_heatField.setValue(1, 4, -1);
        f_heatField.setValue(1, 5, -2);
        f_heatField.setValue(1, 6, -1);
        f_heatField.setValue(2, 1, 4);
        f_heatField.setValue(2, 2, 3);
        f_heatField.setValue(2, 3, 0);
        f_heatField.setValue(2, 4, -1);
        f_heatField.setValue(2, 5, -3);
        f_heatField.setValue(2, 6, -1);
        f_heatField.setValue(3, 1, 3);
        f_heatField.setValue(3, 2, 2);
        f_heatField.setValue(3, 3, -1);
        f_heatField.setValue(3, 4, -2);
        f_heatField.setValue(3, 5, -4);
        f_heatField.setValue(3, 6, -2);

        f_heatFieldCopy = f_heatField;
        
        f_solver.propagateStateExplicitOn(f_heatField, f_delta);

        std::cout << "Value at (1, 3):   " << f_heatField.getValue(2, 4) << std::endl
        << "Value at (0, 3):   " << f_heatField.getValue(1, 4) << std::endl
        << "Value at (0, 5):   " << f_heatField.getValue(1, 6) << std::endl;
    }

    void onDraw(Renderer &renderer) override {
        f_cameraMatrix = renderer.camera.viewMatrix;
        f_fwd = inverse(f_cameraMatrix) * glm::dvec4(0, 0, 1, 0);
        f_right = inverse(f_cameraMatrix) * glm::dvec4(1, 0, 0, 0);
        f_up = inverse(f_cameraMatrix) * glm::dvec4(0, 1, 0, 0);

        f_heatField.onDraw(renderer, f_shift);
        f_heatFieldCopy.onDraw(renderer, f_shift);
    }

    void simulateStep() override {
    
    }

    void onGUI() override {
        ImGui::Text("RMB + drag : to move the graph");
        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            auto drag = ImGui::GetMouseDragDelta(1);
            if(drag.x != 0 || drag.y != 0) {
                glm::dvec3 dx = (double)(drag.x) * f_right;
                glm::dvec3 dy = (double)(-drag.y) * f_up;
                f_shift += 0.01 * (dx + dy);
                ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
            }
        }
    }
};