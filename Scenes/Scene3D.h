#pragma once

#include "Scene.h"
#include "FiniteDifferenceSolver.h"
#include <imgui.h>

class Scene3D : public Scene {
    float f_delta = 0.01;
    bool f_pause = true;
    bool f_singleStep = false;
    float f_diffusivity = 0.1;
    int f_n = 10;
    int f_m = 10;
    int f_p = 10;
    float f_X = 2.;
    float f_Y = 2.;
    float f_Z = 2.;
    bool f_changedProblem = false;
    glm::dvec3 f_shift = glm::dvec3(0.);
    enum Scheme {EXPLICIT, IMPLICIT};
    int f_scheme = Scheme::EXPLICIT;
    FiniteDifferenceSolver::FD_HPRDBC3D_Solver f_solver;
    GridFunction::ScalarGridFunction3D f_heatField;
    bool f_toggleSources = false;
    bool f_sources = false;
    bool f_restart = false;
    int f_effectRadius = 2;

    glm::dmat4 f_cameraMatrix = glm::dmat4(1);
    glm::dvec3 f_fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 f_right = glm::dvec3(0, 1, 0);
    glm::dvec3 f_up = glm::dvec3(0, 0, 1);

public:
    void initializeSolverAndState() {
        f_solver = std::move(FiniteDifferenceSolver::FD_HPRDBC3D_Solver(
            HeatProblem::HeatProblemRectDBC3D(
                glm::dvec3(0., 0., 0.),
                glm::dvec3(f_X, f_Y, f_Z),
                0.1,
                [](glm::dvec3 point, double t) {
                    return 0.;
                },
                [](glm::dvec3 point, double t) {
                    return 10 * ((double)(rand()) / (double)(RAND_MAX)) - 5;
                },
                [](glm::dvec2 point, double t) {
                    return 0;
                },
                [](glm::dvec2 point, double t) {
                    return 0;
                },
                [](glm::dvec2 point, double t) {
                    return 0;
                },
                [](glm::dvec2 point, double t) {
                    return 0;
                },
                [](glm::dvec2 point, double t) {
                    return 0;
                },
                [](glm::dvec2 point, double t) {
                    return 0;
                }
            )
        ));
        f_heatField = f_solver.getInitialState(f_n, f_m, f_p);
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

    void propagateState() {
        switch (f_scheme)
        {
        case Scheme::EXPLICIT:
            f_solver.propagateStateExplicitOn(f_heatField, f_delta);
            break;
        case Scheme::IMPLICIT:
            f_solver.propagateStateImplicitOn(f_heatField, f_delta);
            break;
        default:
            break;
        }
    }

    void simulateStep() override {
        if (f_changedProblem || f_restart) {
            f_pause = true;
            initializeSolverAndState();
            f_sources = false;
            f_changedProblem = false;
            f_restart = false;
        }
        if (f_toggleSources) {
            if (!f_sources) {
                f_solver.setSources(
                    [](glm::dvec3 point, double t) {
                        return 0.;
                    }
                );
            }
            else {
                f_solver.setSources(
                    [](glm::dvec3 point, double t) {
                        return 0.05 * (sin(6.28 * point.x) * sin(6.28 * point.y) * sin(6.28 * point.z) + 1.);
                    }
                );
            }
            f_toggleSources = false;
        }
        f_solver.setDiffusivity(f_diffusivity);
        if (!f_pause) {
            propagateState();
        }
        else {
            if (f_singleStep) {
                propagateState();
                f_singleStep = false;
            }
        }
    }

    void onGUI() override {
        ImGui::SliderFloat("Diffusivity", &this->f_diffusivity, 0.001f, 1.);
        f_changedProblem = f_changedProblem || ImGui::SliderFloat("Domain size X", &this->f_X, 0.1, 10.);
        f_changedProblem = f_changedProblem || ImGui::SliderFloat("Domain size Y", &this->f_Y, 0.1, 10.);
        f_changedProblem = f_changedProblem || ImGui::SliderFloat("Domain size Z", &this->f_Z, 0.1, 10.);
        f_toggleSources = ImGui::Checkbox("Heat sources", &this->f_sources);
        const char* schemeNames[] = {"Explicit", "Implicit"};
        ImGui::Combo("Scheme", &this->f_scheme, schemeNames, 2);
        ImGui::Text("N, M and P denote the overall amount");
        ImGui::Text("of vertices along Ox, Oy and Oz respectively,");
        ImGui::Text("including the boundary vertices");
        f_changedProblem = f_changedProblem || ImGui::SliderInt("N", &this->f_n, 3, 30);
        f_changedProblem = f_changedProblem || ImGui::SliderInt("M", &this->f_m, 3, 30);
        f_changedProblem = f_changedProblem || ImGui::SliderInt("P", &this->f_p, 3, 30);
        ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
        ImGui::Checkbox("Pause", &this->f_pause);
        ImGui::Text("Space : pause/unpause");
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->f_pause = !this->f_pause;
        }
        f_restart = ImGui::Button("Restart");
        ImGui::Text("S : while paused perform a single time step");
        if (ImGui::IsKeyPressed(ImGuiKey_S)) {
            this->f_singleStep = true;
        }
        ImGui::Text("W : Toggle heat source");
        if (ImGui::IsKeyPressed(ImGuiKey_W)) {
            this->f_sources = !f_sources;
            f_toggleSources = true;
        }
        ImGui::SliderInt("Manual effect radius", &this->f_effectRadius, 0, 5);
        ImGui::Text("While unpaused:");
        ImGui::Text("Hold E : Heat up depending on mouse cursor position");
        if (ImGui::IsKeyDown(ImGuiKey_E)) {
            if (!f_pause) {
                auto mousePosition = ImGui::GetMousePos();
                auto windowSize = ImGui::GetIO().DisplaySize;
                double ty = (double)(mousePosition.x) / (double)(windowSize.x);
                double tz = 1 - (double)(mousePosition.y) / (double)(windowSize.y);
                glm::dvec3 tp(0.5, ty, tz);
                int i = (int)(f_heatField.getN() * tp.x);
                int j = (int)(f_heatField.getM() * tp.y);
                int k = (int)(f_heatField.getP() * tp.z);
                for (int io = std::max(0, i - f_effectRadius); io <= i + f_effectRadius; io++) {
                    for (int jo = std::max(0, j - f_effectRadius); jo <= j + f_effectRadius; jo++) {
                        for (int ko = std::max(0, k - f_effectRadius); ko <= k + f_effectRadius; ko++) {
                            f_heatField.addToValue(io, jo, ko, 0.5);
                        }
                    }
                }
            }
        }
        ImGui::Text("Hold Q : Cool down depending on mouse cursor position");
        if (ImGui::IsKeyDown(ImGuiKey_Q)) {
            if (!f_pause) {
                auto mousePosition = ImGui::GetMousePos();
                auto windowSize = ImGui::GetIO().DisplaySize;
                double ty = (double)(mousePosition.x) / (double)(windowSize.x);
                double tz = 1 - (double)(mousePosition.y) / (double)(windowSize.y);
                glm::dvec3 tp(0.5, ty, tz);
                int i = (int)(f_heatField.getN() * tp.x);
                int j = (int)(f_heatField.getM() * tp.y);
                int k = (int)(f_heatField.getP() * tp.z);
                for (int io = std::max(0, i - f_effectRadius); io <= i + f_effectRadius; io++) {
                    for (int jo = std::max(0, j - f_effectRadius); jo <= j + f_effectRadius; jo++) {
                        for (int ko = std::max(0, k - f_effectRadius); ko <= k + f_effectRadius; ko++) {
                            f_heatField.addToValue(io, jo, ko, -0.5);
                        }
                    }
                }
            }
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