#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"

class SceneSingleStep : public Scene {
private:
    std::shared_ptr<Box> initialStateCopy;
    Force* forceCopy;
    RigidObjectPhysicsEngine ROPE;

public:
    SceneSingleStep():
    ROPE()
    {}

    ~SceneSingleStep() {
        initialStateCopy.reset();
        delete forceCopy;
    }

    void init() override {
        std::shared_ptr<Box> box1 (new Box(
            2.,
            0.,
            0.25,
            std::move(glm::dvec3(0.)),
            std::move(glm::dvec3(1., 0.6, 0.5)),
            std::move(glm::dvec3(0., 0., glm::pi<double>() / 2.)),
            std::move(glm::dvec3(0.)),
            std::move(glm::dvec3(0.))
        ));

        initialStateCopy = box1;
        
        ROPE.addObject(box1);

        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;

        forceCopy = new Force(
            std::move(glm::dvec3(1., 1., 0.)),
            std::move(glm::dvec3(0.3, 0.5, 0.25))
        );

        ROPE.applyForceToObject(
            0, 
            *forceCopy
        );

        double delta = 2.;
        ROPE.euIntegrate(delta);

        std::cout << "======== Final system state ========" << std::endl
        << ROPE << std::endl << std::endl;
    }

    void onDraw(Renderer &renderer) override {
        renderer.drawWireCube(glm::dvec3(0), glm::dvec3(5), glm::dvec3(1));
        initialStateCopy->onDraw(renderer);
        forceCopy->onDraw(renderer);
        ROPE.onDraw(renderer);
    }
};