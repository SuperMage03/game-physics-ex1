#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include <imgui.h>

class SceneRacket : public Scene {
private:
    RigidObjectPhysicsEngine ROPE;
    float delta = 0.01;
    bool pause = true;

    glm::dmat4 cameraMatrix = glm::dmat4(1);
    glm::dvec3 fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 right = glm::dvec3(0, 1, 0);
    glm::dvec3 up = glm::dvec3(0, 0, 1);

public:
    SceneRacket():
    ROPE()
    {}

    void init() override {
        ROPE.f_integrationType = IntegrationType::MIDPOINT;

        std::shared_ptr<Box> box1 (new Box(
            8., // mass
            0.5, // c
            1., // mu
            std::move(glm::dvec3(0.)), // pos
            std::move(glm::dvec3(2., 1.2, 0.3)), // scale
            std::move(glm::dvec3(0., 0., 0.)), // angles
            std::move(glm::dvec3(0., 0., 0.)), // velocity
            std::move(glm::dvec3(0., 5., 0.0000001)) // angular velocity
        ));

        box1->f_color = glm::vec4(0.9f, 0.9f, 0.9f, 1.f);

        ROPE.addObject(box1);

        // Add walls
        Wall Xm(glm::dvec3(1., 0., 0.), -2.5);
        Wall Xp(glm::dvec3(-1., 0., 0.), -2.5);
        Wall Ym(glm::dvec3(0., 1., 0.), -2.5);
        Wall Yp(glm::dvec3(0., -1., 0.), -2.5);
        Wall Zm(glm::dvec3(0., 0., 1.), -2.5);
        Wall Zp(glm::dvec3(0., 0., -1.), -2.5);
        ROPE.addWall(std::move(Xm));
        ROPE.addWall(std::move(Xp));
        ROPE.addWall(std::move(Ym));
        ROPE.addWall(std::move(Yp));
        ROPE.addWall(std::move(Zm));
        ROPE.addWall(std::move(Zp));

        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;
    }

    void simulateStep() override {
        if (!pause) {
            ROPE.simulateStep(delta);

            std::cout << "    <^> Kinetic energy:" << std::setprecision(10)
		    << glm::dot(ROPE.getObject(0).f_angularVelocity, ROPE.getObject(0).f_inertiaTensorInv * ROPE.getObject(0).f_angularVelocity) / 2. << std::endl;
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

        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){   
            auto drag = ImGui::GetMouseDragDelta(1);
            if(drag.x != 0 || drag.y != 0) {
                glm::dvec3 dx = (double)(drag.x) * right;
                glm::dvec3 dy = (double)(-drag.y) * up;
                ROPE.applyForceToObject(
                    0,
                    Force (
                        (dx + dy) / 50.,
                        ROPE.getObject(0).f_transform.f_position
                    )
                );
            }
        }
    }
};