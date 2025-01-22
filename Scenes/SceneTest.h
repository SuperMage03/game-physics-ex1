#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include "ThermodynamicPhysicsEngine.h"
#include <imgui.h>

class SceneTest : public Scene {
private:
    Physics::RigidObjectPhysicsEngine ROPE;
    float f_delta = 0.01;
    bool f_pause = true;
    bool gravity = false;
    Physics::IntegrationType f_integrationType = Physics::IntegrationType::MIDPOINT;

    bool f_singleStep = false;
    float f_diffusivity = 0.1;
    int f_n = 50;
    int f_m = 50;
    float f_X = 5.;
    float f_Y = 5.;
    bool f_changedProblem = false;
    Physics::SolverType f_solverType = Physics::SolverType::IMPLICIT;
    int f_effectRadius = 2;

    Physics::ThermodynamicPhysicsEngine TPE;
    GridFunction::ScalarGridFunction2D f_heatField;

    bool f_restart = false;


    glm::dmat4 cameraMatrix = glm::dmat4(1);
    glm::dvec3 fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 right = glm::dvec3(0, 1, 0);
    glm::dvec3 up = glm::dvec3(0, 0, 1);

public:
    SceneTest():
    ROPE(),
    TPE()
    {}

    void init() override {
        initialize();
    }

    void initialize() {
        ROPE = Physics::RigidObjectPhysicsEngine();
        ROPE.f_integrationType = f_integrationType;

        std::shared_ptr<RigidBall> ball1(new RigidBall(
            1.,
            0.8,
            0.15,
            Transform3D(
                glm::dvec3(0.5, 0, 0.),
                glm::dvec3(0.4, 0.4, 0.4),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec3(0., 0., 0.),
            glm::dvec3(10., 0., 0.)
        ));

        // std::shared_ptr<RigidBall> ball2(new RigidBall(
        //     1.,
        //     1.0,
        //     0.0,
        //     Transform3D(
        //         glm::dvec3(0.5, 0., 0.),
        //         glm::dvec3(0.4, 0.4, 0.4),
        //         glm::dvec3(0., 0., 0.)
        //     ),
        //     glm::dvec3(0., 0., 0.),
        //     glm::dvec3(0., 0., 0.)
        // ));

        std::shared_ptr<StaticCuboid> wallB(new StaticCuboid(
            Transform3D(
                glm::dvec3(0., 0., -3.),
                glm::dvec3(f_X, f_Y, 1.),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec4(0.8, 0.8, 0.8, 0.8)
        ));

        ball1->f_color = glm::dvec4(1.0, 0.1, 0.05, 1.);
        // ball2->f_color = glm::dvec4(0.1, 1.0, 0.05, 1.);

        ROPE.addRigidObject(ball1);
        // ROPE.addRigidObject(ball2);
        
        ROPE.addStaticObject(wallB);

        ROPE.setGridFunction(&f_heatField);
        
        std::cout << "+++++++++++ SCENE TEST +++++++++++" << std::endl;
        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;

        TPE = std::move(Physics::ThermodynamicPhysicsEngine(
            // HeatProblem::HeatProblemRectDBC2D(
            //     glm::dvec2(-f_X / 2., -f_Y / 2.),
            //     glm::dvec2(f_X, f_Y),
            //     0.1,
            //     [](glm::dvec2 point, double t) {
            //         return 0.;
            //     },
            //     [](glm::dvec2 point, double t) {
            //         return 2 * sin(2. * point.x) * sin(2. * point.y) + 0.5 * ((double)(rand()) / (double)(RAND_MAX)) - 2.5;
            //     },
            //     [](double y, double t) {
            //         return 2. * (sin(y - t) + 1.) + 0.5 * (sin(5. * (y + t)) + 1.) - 2.5;
            //     },
            //     [](double y, double t) {
            //         return sin(t) * y - 2.5;
            //     },
            //     [](double x, double t) {
            //         return 2. * (sin(x - t) + 1.) + 0.5 * (sin(5. * (x + t)) + 1.) - 2.5;
            //     },
            //     [](double x, double t) {
            //         return sin(t) * x - 2.5;
            //     },
            //     HeatProblem::BoundaryConditionType::DIRICHLET,
            //     HeatProblem::BoundaryConditionType::DIRICHLET,
            //     HeatProblem::BoundaryConditionType::DIRICHLET,
            //     HeatProblem::BoundaryConditionType::DIRICHLET
            // )
            HeatProblem::HeatProblemRectDBC2D(
                glm::dvec2(-f_X / 2., -f_Y / 2.),
                glm::dvec2(f_X, f_Y),
                0.1,
                [](glm::dvec2 point, double t) {
                    return 0.;
                },
                [](glm::dvec2 point, double t) {
                    return - 4. * (point.x - 2.5) * (point.x + 2.5) * (point.y - 2.5) * (point.y + 2.5) / 39. + 2.5;
                },
                [](double y, double t) {
                    return 2.5;
                },
                [](double y, double t) {
                    return 2.5;
                },
                [](double x, double t) {
                    return 2.5;
                },
                [](double x, double t) {
                    return 2.5;
                },
                HeatProblem::BoundaryConditionType::DIRICHLET,
                HeatProblem::BoundaryConditionType::DIRICHLET,
                HeatProblem::BoundaryConditionType::DIRICHLET,
                HeatProblem::BoundaryConditionType::DIRICHLET
            )
        ));
        TPE.f_solverType = f_solverType;

        f_heatField = TPE.getInitialState(f_n, f_m);
    }

    void propagateState() {
        // switch (f_solverType) {
        // case Physics::SolverType::EXPLICIT:
        //     TPE.propagateStateExplicitOn(f_heatField, f_delta);
        //     break;
        // case Physics::SolverType::IMPLICIT:
        //     TPE.propagateStateImplicitOn(f_heatField, f_delta);
        //     break;
        // default:
        //     break;
        // }
    }

    void simulateStep() override {
        if (f_changedProblem || f_restart) {
            f_pause = true;
            initialize();
            f_changedProblem = false;
            f_restart = false;
        }
        TPE.setDiffusivity(f_diffusivity);
        if (!f_pause) {
            if (gravity) {
                ROPE.applyForceToRigidObject(0, Force(glm::dvec3(0.0, 0.0, -9.81), ROPE.getRigidObject(0)->f_transform.f_position));
                // ROPE.applyForceToRigidObject(1, Force(glm::dvec3(0.0, 0.0, -9.81), ROPE.getRigidObject(1)->f_transform.f_position));
            }
            ROPE.simulateStep(f_delta);
            propagateState();
        }
        else {
            if (f_singleStep) {
                propagateState();
                f_singleStep = false;
            }
        }
    }

    void onDraw(Renderer &renderer) override {
        cameraMatrix = renderer.camera.viewMatrix;
        fwd = inverse(cameraMatrix) * glm::dvec4(0, 0, 1, 0);
        right = inverse(cameraMatrix) * glm::dvec4(1, 0, 0, 0);
        up = inverse(cameraMatrix) * glm::dvec4(0, 1, 0, 0);

        ROPE.onDraw(renderer);
        f_heatField.onDraw(renderer);
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
        ImGui::Checkbox("Pause", &this->f_pause);
        ImGui::Text("Space : f_pause/unpause");
        ImGui::Checkbox("Gravity", &this->gravity);
        ImGui::Text("RMB + drag : apply force to one of the objects.");
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->f_pause = !this->f_pause;
        }
        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){   
            auto drag = ImGui::GetMouseDragDelta(1);
            if(drag.x != 0 || drag.y != 0) {
                glm::dvec3 dx = (double)(drag.x) * right;
                glm::dvec3 dy = (double)(-drag.y) * up;
                ROPE.applyForceToRigidObject(
                    0,
                    Force (
                        (dx + dy) / 25.,
                        ROPE.getRigidObject(0)->f_transform.f_position
                    )
                );
            }
        }
        ImGui::SliderFloat("Diffusivity", &this->f_diffusivity, 0.001f, 1.);
        const char* schemeNames[] = {"Explicit", "Implicit"};
        ImGui::Text("N and M denote the overall amount");
        ImGui::Text("of vertices along Ox and Oy respectively,");
        ImGui::Text("including the boundary vertices");
        f_changedProblem = f_changedProblem || ImGui::SliderInt("N", &this->f_n, 3, 100);
        f_changedProblem = f_changedProblem || ImGui::SliderInt("M", &this->f_m, 3, 100);
        f_restart = ImGui::Button("Restart");
        ImGui::Text("R : Restart");
        if (ImGui::IsKeyPressed(ImGuiKey_R)) {
            f_restart = true;
        }
        ImGui::Text("S : while paused perform a single time step");
        if (ImGui::IsKeyPressed(ImGuiKey_S)) {
            this->f_singleStep = true;
        }
        ImGui::SliderInt("Manual effect radius", &this->f_effectRadius, 0, 5);
        ImGui::Text("While unpaused:");
        ImGui::Text("Hold E : Heat up depending on mouse cursor position");
        if (ImGui::IsKeyDown(ImGuiKey_E)) {
            if (!f_pause) {
                auto mousePosition = ImGui::GetMousePos();
                auto windowSize = ImGui::GetIO().DisplaySize;
                double tx = (double)(mousePosition.x) / (double)(windowSize.x);
                double ty = 1 - (double)(mousePosition.y) / (double)(windowSize.y);
                glm::dvec2 tp(tx, ty);
                int i = (int)(f_heatField.getN() * tp.x);
                int j = (int)(f_heatField.getM() * tp.y);
                for (int io = std::max(0, i - f_effectRadius); io <= i + f_effectRadius; io++) {
                    for (int jo = std::max(0, j - f_effectRadius); jo <= j + f_effectRadius; jo++) {
                        f_heatField.addToValue(io, jo, 0.5);
                    }
                }
            }
        }
        ImGui::Text("Hold Q : Cool down depending on mouse cursor position");
        if (ImGui::IsKeyDown(ImGuiKey_Q)) {
            if (!f_pause) {
                auto mousePosition = ImGui::GetMousePos();
                auto windowSize = ImGui::GetIO().DisplaySize;
                double tx = (double)(mousePosition.x) / (double)(windowSize.x);
                double ty = 1 - (double)(mousePosition.y) / (double)(windowSize.y);
                glm::dvec2 tp(tx, ty);
                int i = (int)(f_heatField.getN() * tp.x);
                int j = (int)(f_heatField.getM() * tp.y);
                for (int io = std::max(0, i - f_effectRadius); io <= i + f_effectRadius; io++) {
                    for (int jo = std::max(0, j - f_effectRadius); jo <= j + f_effectRadius; jo++) {
                        f_heatField.addToValue(io, jo, -0.5);
                    }
                }
            }
        }
    }
};