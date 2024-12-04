#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include <imgui.h>

class SceneComplexSimulation : public Scene {
private:
    RigidObjectPhysicsEngine ROPE;
    float delta = 0.01;
    bool pause = true;

    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

public:
    SceneComplexSimulation():
    ROPE()
    {}

    void init() override {
        ROPE.f_integrationType = ROPE.EULER;

        Box* box1 = new Box(
            2.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(-1.f, 0.f, 0.f)), // pos
            std::move(glm::vec3(0.6f, 0.6f, 0.6f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 0.f, 0.f)) // angular velocity
        );
        Box* box2 = new Box(
            2.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(1.f, 0.f, 0.f)), // pos
            std::move(glm::vec3(0.6f, 0.6f, 0.6f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 0.f, 0.f)) // angular velocity
        );
        Box* box3 = new Box(
            2.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(0.f, -1.f, 0.f)), // pos
            std::move(glm::vec3(0.6f, 0.6f, 0.6f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 0.f, 0.f)) // angular velocity
        );
        Box* box4 = new Box(
            2.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(0.f, 1.f, 0.f)), // pos
            std::move(glm::vec3(0.6f, 0.6f, 0.6f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 0.f, 0.f)) // angular velocity
        );
        Box* box5 = new Box(
            2.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(0.f, 0.f, -1.f)), // pos
            std::move(glm::vec3(0.6f, 0.6f, 0.6f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 0.f, 0.f)) // angular velocity
        );
        Box* box6 = new Box(
            2.f, // mass
            0.5f, // c
            1.f, // mu
            std::move(glm::vec3(0.f, 0.f, 1.f)), // pos
            std::move(glm::vec3(0.6f, 0.6f, 0.6f)), // scale
            std::move(glm::vec3(0.f, 0.f, 0.f)), // angles
            std::move(glm::vec3(0.f, 0.f, 0.f)), // velocity
            std::move(glm::vec3(0.f, 0.f, 0.f)) // angular velocity
        );
        

        box1->f_color = glm::vec4(0.9f, 0.9f, 0.9f, 1.f);
        box2->f_color = glm::vec4(1.f, 0.15f, 0.15f, 1.f);
        box3->f_color = glm::vec4(0.15f, 1.f, 0.15f, 1.f);
        box4->f_color = glm::vec4(0.15f, 0.15f, 1.f, 1.f);
        box5->f_color = glm::vec4(0.8f, 0.8f, 0.15f, 1.f);
        box6->f_color = glm::vec4(0.15f, 0.8f, 0.8f, 1.f);

        ROPE.addObject(box1);
        ROPE.addObject(box2);
        ROPE.addObject(box3);
        ROPE.addObject(box4);
        ROPE.addObject(box5);
        ROPE.addObject(box6);

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