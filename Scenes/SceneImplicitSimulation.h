#pragma once

#include "Scene.h"
#include "FiniteDifferenceSolver.h"
#include <imgui.h>

class SceneImplicitSimulation : public Scene {
    float f_delta = 0.01;
    bool f_pause = true;
    bool f_singleStep = false;
    float f_diffusivity = 0.1;
    unsigned f_n = 18;
    unsigned f_m = 18;
    glm::dvec3 f_shift = glm::dvec3(0.);
    FiniteDifferenceSolver::FD_HPRDBC2D_Solver f_solver;
    GridFunction::ScalarGridFunction2D f_heatField;
    bool f_restart = false;

    glm::dmat4 f_cameraMatrix = glm::dmat4(1);
    glm::dvec3 f_fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 f_right = glm::dvec3(0, 1, 0);
    glm::dvec3 f_up = glm::dvec3(0, 0, 1);

public:
    void initializeSolverAndState() {
        f_solver = std::move(FiniteDifferenceSolver::FD_HPRDBC2D_Solver(
            HeatProblem::HeatProblemRectDBC2D(
                glm::dvec2(0., 0.),
                glm::dvec2(1., 1.),
                0.1,
                [](glm::dvec2 point, double t) {
                    return 0.;
                },
                [](glm::dvec2 point, double t) {
                    return 2 * sin(4. * point.x) * sin(4. * point.y) + 0.5 * ((double)(rand()) / (double)(RAND_MAX));
                },
                [](double y, double t) {
                    return 0.;
                },
                [](double y, double t) {
                    return 0.;
                },
                [](double x, double t) {
                    return 0.;
                },
                [](double x, double t) {
                    return 0.;
                }
            )
        ));
        f_heatField = f_solver.getInitialState(f_n, f_m);
    }

    void init() override {
        initializeSolverAndState();
    }

    void onDraw(Renderer &renderer) override {
        f_cameraMatrix = renderer.camera.viewMatrix;
        f_fwd = inverse(f_cameraMatrix) * glm::dvec4(0, 0, 1, 0);
        f_right = inverse(f_cameraMatrix) * glm::dvec4(1, 0, 0, 0);
        f_up = inverse(f_cameraMatrix) * glm::dvec4(0, 1, 0, 0);

        f_heatField.onDraw(renderer, f_shift);
    }

    void simulateStep() override {
        if (f_restart) {
            f_pause = true;
            initializeSolverAndState();
            f_restart = false;
        }
        f_solver.setDiffusivity(f_diffusivity);
        if (!f_pause) {
            f_solver.propagateStateImplicitOn(f_heatField, f_delta);
        }
        else {
            if (f_singleStep) {
                f_solver.propagateStateImplicitOn(f_heatField, f_delta);
                f_singleStep = false;
            }
        }
    }

    void onGUI() override {
        ImGui::SliderFloat("Diffusivity", &this->f_diffusivity, 0.001f, 1.);
        ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
        ImGui::Checkbox("Pause", &this->f_pause);
        ImGui::Text("Space : pause/unpause");
        f_restart = ImGui::Button("Restart");
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->f_pause = !this->f_pause;
        }
        ImGui::Text("S : while paused perform a single time step");
        if (ImGui::IsKeyPressed(ImGuiKey_S)) {
            this->f_singleStep = true;
        }
        ImGui::Text("R : Restart");
        if (ImGui::IsKeyPressed(ImGuiKey_R)) {
            f_restart = true;
        }
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