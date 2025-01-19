#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include <imgui.h>

class SceneTest : public Scene {
private:
    Physics::RigidObjectPhysicsEngine ROPE;
    float delta = 0.01;
    bool pause = true;
    bool gravity = false;

    glm::dmat4 cameraMatrix = glm::dmat4(1);
    glm::dvec3 fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 right = glm::dvec3(0, 1, 0);
    glm::dvec3 up = glm::dvec3(0, 0, 1);

public:
    SceneTest():
    ROPE()
    {}

    void init() override {
        ROPE.f_integrationType = Physics::IntegrationType::MIDPOINT;

        std::shared_ptr<RigidBall> ball1(new RigidBall(
            1.,
            0.85,
            0.0,
            Transform3D(
                glm::dvec3(-0.5, 0., 0.),
                glm::dvec3(0.4, 0.4, 0.4),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec3(0., 0., 0.),
            glm::dvec3(0., 0., 0.)
        ));

        std::shared_ptr<RigidBall> ball2(new RigidBall(
            1.,
            0.85,
            0.0,
            Transform3D(
                glm::dvec3(0.5, 0., 0.),
                glm::dvec3(0.4, 0.4, 0.4),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec3(0., 0., 0.),
            glm::dvec3(0., 0., 0.)
        ));

        std::shared_ptr<StaticCuboid> wallB(new StaticCuboid(
            Transform3D(
                glm::dvec3(0., 0., -3.),
                glm::dvec3(10., 10., 1.),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec4(0.8, 0.8, 0.8, 0.8)
        ));

        ball1->f_color = glm::dvec4(1.0, 0.1, 0.05, 1.);
        ball2->f_color = glm::dvec4(0.1, 1.0, 0.05, 1.);

        ROPE.addRigidObject(ball1);
        ROPE.addRigidObject(ball2);
        
        ROPE.addStaticObject(wallB);
        
        std::cout << "+++++++++++ SCENE TEST +++++++++++" << std::endl;
        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;
    }

    void simulateStep() override {
        if (!pause) {
            if (gravity) {
                ROPE.applyForceToRigidObject(0, Force(glm::dvec3(0.0, 0.0, -9.81), ROPE.getRigidObject(0)->f_transform.f_position));
            }
            ROPE.simulateStep(delta);
        }
    }

    void onDraw(Renderer &renderer) override {
        cameraMatrix = renderer.camera.viewMatrix;
        fwd = inverse(cameraMatrix) * glm::dvec4(0, 0, 1, 0);
        right = inverse(cameraMatrix) * glm::dvec4(1, 0, 0, 0);
        up = inverse(cameraMatrix) * glm::dvec4(0, 1, 0, 0);

        renderer.drawWireCube(glm::dvec3(0), glm::dvec3(5), glm::dvec3(1));
        ROPE.onDraw(renderer);
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
                ROPE.applyForceToRigidObject(
                    0,
                    Force (
                        (dx + dy) / 25.,
                        ROPE.getRigidObject(0)->f_transform.f_position
                    )
                );
            }
        }
    }
};