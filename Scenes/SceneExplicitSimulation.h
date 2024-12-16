#pragma once

#include "Scene.h"
#include "FiniteDifferenceSolver.h"
#include <imgui.h>

class SceneExplicitSimulation : public Scene {
    float f_delta = 0.01;
    bool f_pause = true;
    FiniteDifferenceSolver::FD_HPRDBC2D_Solver f_solver;
    GridFunction::ScalarGridFunction2D f_heatField;

public:
    SceneExplicitSimulation()
    {}

    void init() override {
        f_solver = std::move(FiniteDifferenceSolver::FD_HPRDBC2D_Solver(
            HeatProblem::HeatProblemRectDBC2D(
                glm::dvec2(0., 0.),
                glm::dvec2(1., 1.),
                0.1,
                [](glm::dvec2 point) {
                    return (2. * (double)(rand()) / (double)(RAND_MAX));
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
        f_heatField = f_solver.getInitialState(18, 18);
    }

    void onDraw(Renderer &renderer) override {
        f_heatField.onDraw(renderer);
    }

    void simulateStep() override {
        if (!f_pause) {
            f_solver.propagateStateExplicitOn(f_heatField, f_delta);
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