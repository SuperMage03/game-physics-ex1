#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include <imgui.h>

class SceneSimulation : public Scene {
private:
    Force forceCopy;
    RigidObjectPhysicsEngine ROPE;
    float delta = 0.01;
    bool pause = true;

public:
    SceneSimulation():
    ROPE()
    {}

    void init() override {
        Box* box1 = new Box(
            2.f,
            std::move(glm::vec3(0.f)),
            std::move(glm::vec3(1.f, 0.6f, 0.5f)),
            std::move(glm::vec3(0.f, 0.f, glm::pi<float>() / 2.f)),
            std::move(glm::vec3(0.f)),
            std::move(glm::vec3(0.f))
        );

        ROPE.addObject(box1);

        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;

        forceCopy = Force(
            std::move(glm::vec3(1.f, 1.f, 0.f)),
            std::move(glm::vec3(0.3f, 0.5f, 0.25f))
        );
    }

    void simulateStep() override {
        if (!pause) {
            // ROPE.applyForceToObject(
            //     0, 
            //     *forceCopy
            // );
            Force localForce(
                std::move(glm::vec3(1.f, 0.f, 0.f)),
                std::move(glm::vec3(0.f, 0.5f, 0.f))
            );
            forceCopy = ROPE.getObject(0).f_transform.transformLocalForceToGlobalForce(localForce);
            ROPE.applyLocalForceToObject(
                0,
                localForce
            );
            ROPE.lfIntegrate(delta);
        }
    }

    void onDraw(Renderer &renderer) override {
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
        forceCopy.onDraw(renderer);
        ROPE.onDraw(renderer);
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1f);
        ImGui::Checkbox("Pause", &this->pause);
    }
};