#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include <imgui.h>

class SceneRacket : public Scene {
private:
    RigidObjectPhysicsEngine ROPE;
    float delta = 0.01;
    bool pause = true;

    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

public:
    SceneRacket():
    ROPE()
    {}

    void init() override {
        ROPE.f_integrationType = ROPE.MIDPOINT;

        Box* box1 = new Box(
            8.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(0.f)), // pos
            std::move(glm::vec3(2.f, 1.2f, 0.3f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 5.f, 0.0000001f)) // angular velocity
        );

        box1->f_color = glm::vec4(0.9f, 0.9f, 0.9f, 1.f);

        ROPE.addObject(box1);

        // Add walls
        Wall Xm(glm::vec3(1.f, 0.f, 0.f), -2.5f);
        Wall Xp(glm::vec3(-1.f, 0.f, 0.f), -2.5f);
        Wall Ym(glm::vec3(0.f, 1.f, 0.f), -2.5f);
        Wall Yp(glm::vec3(0.f, -1.f, 0.f), -2.5f);
        Wall Zm(glm::vec3(0.f, 0.f, 1.f), -2.5f);
        Wall Zp(glm::vec3(0.f, 0.f, -1.f), -2.5f);
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
            // TEST RENORMALIZE ANGULAR VELOCITY
            // ROPE.getObject(0).f_angularVelocity = glm::normalize(ROPE.getObject(0).f_angularVelocity) * 8.f;
            // std::cout << "    <^> Angular velocity:   ("
		    // << ROPE.getObject(0).f_angularVelocity[0] << "; "
		    // << ROPE.getObject(0).f_angularVelocity[1] << "; "
		    // << ROPE.getObject(0).f_angularVelocity[2] << ")" << std::endl
            // << "    <^> Angular velocity length:   "
            // << glm::length(ROPE.getObject(0).f_angularVelocity) << "; " << std::endl;

            // glm::vec3 angularMomentum = glm::inverse(ROPE.getObject(0).f_inertiaTensorInv) * ROPE.getObject(0).f_angularVelocity;
            // std::cout << "    <^> Angular momentum:   ("
		    // << angularMomentum[0] << "; "
		    // << angularMomentum[1] << "; "
		    // << angularMomentum[2] << ")" << std::endl
            // << "    <^> Angular momentum length:   "
            // << glm::length(angularMomentum) << "; " << std::endl << std::endl;

            std::cout << "    <^> Kinetic energy:" << std::setprecision(10)
		    << glm::dot(ROPE.getObject(0).f_angularVelocity, ROPE.getObject(0).f_inertiaTensorInv * ROPE.getObject(0).f_angularVelocity) / 2.f << std::endl;
            // std::cout << "    <^> Inertia tensor inv determinant:   "
            // << glm::determinant(ROPE.getObject(0).f_inertiaTensorInv) << "; " << std::endl << std::endl;
            // std::cout << "    <^> Iw:   ("
		    // << (glm::inverse(ROPE.getObject(0).f_inertiaTensorInv) * ROPE.getObject(0).f_angularVelocity)[0] << "; "
		    // << (glm::inverse(ROPE.getObject(0).f_inertiaTensorInv) * ROPE.getObject(0).f_angularVelocity)[1] << "; "
		    // << (glm::inverse(ROPE.getObject(0).f_inertiaTensorInv) * ROPE.getObject(0).f_angularVelocity)[2] << ")" << std::endl;
        }
    }

    void onDraw(Renderer &renderer) override {
        cameraMatrix = renderer.camera.viewMatrix;
        fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
        right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
        up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);

        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
        ROPE.onDraw(renderer);
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1f);
        ImGui::Checkbox("Pause", &this->pause);

        if(ImGui::IsMouseReleased(ImGuiMouseButton_Right)){   
            auto drag = ImGui::GetMouseDragDelta(1);
            if(drag.x != 0 || drag.y != 0) {
                glm::vec3 dx = drag.x * right;
                glm::vec3 dy = -drag.y * up;
                ROPE.applyForceToObject(
                    0,
                    Force (
                        (dx + dy) / 5.f,
                        ROPE.getObject(0).f_transform.f_position + ROPE.getObject(0).f_transform.f_scale[0] * fwd / 2.f
                    )
                );
            }
        }
    }
};