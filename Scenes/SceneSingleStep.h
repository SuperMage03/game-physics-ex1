#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"

class SceneSingleStep : public Scene {
private:
    Box* initialStateCopy;
    Force* forceCopy;
    RigidObjectPhysicsEngine ROPE;

public:
    SceneSingleStep():
    ROPE()
    {}

    void init() override {
        initialStateCopy = new Box(
            2.f,
            0.f,
            0.5f,
            std::move(glm::vec3(0.f)),
            std::move(glm::vec3(1.f, 0.6f, 0.5f)),
            std::move(glm::vec3(0.f, 0.f, glm::pi<float>() / 2.f)),
            std::move(glm::vec3(0.f)),
            std::move(glm::vec3(0.f))
        );

        Box* box1 = new Box(
            2.f,
            0.f,
            0.5f,
            std::move(glm::vec3(0.f)),
            std::move(glm::vec3(1.f, 0.6f, 0.5f)),
            std::move(glm::vec3(0.f, 0.f, glm::pi<float>() / 2.f)),
            std::move(glm::vec3(0.f)),
            std::move(glm::vec3(0.f))
        );
        ROPE.addObject(box1);

        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;

        forceCopy = new Force(
            std::move(glm::vec3(1.f, 1.f, 0.f)),
            std::move(glm::vec3(0.3f, 0.5f, 0.25f))
        );

        ROPE.applyForceToObject(
            0, 
            *forceCopy
        );

        float delta = 2.f;
        ROPE.euIntegrate(delta);

        std::cout << "======== Final system state ========" << std::endl
        << ROPE << std::endl << std::endl;
    }

    void onDraw(Renderer &renderer) override {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
        initialStateCopy->onDraw(renderer);
        forceCopy->onDraw(renderer);
        ROPE.onDraw(renderer);
    }
};