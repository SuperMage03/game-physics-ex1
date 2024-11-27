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
            std::move(glm::vec3(5.f, 5.f, 0.f))
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
            //     forceCopy
            // );
            // ROPE.euIntegrate(delta);
            ROPE.mpIntegrate(delta);
            // TEST RENORMALIZE ANGULAR VELOCITY
            // ROPE.getObject(0).f_angularVelocity = glm::normalize(ROPE.getObject(0).f_angularVelocity) * 8.f;
            // std::cout << "    <^> Angular velocity:   ("
		    // << ROPE.getObject(0).f_angularVelocity[0] << "; "
		    // << ROPE.getObject(0).f_angularVelocity[1] << "; "
		    // << ROPE.getObject(0).f_angularVelocity[2] << ")" << std::endl
            // << "    <^> Angular velocity length:   "
            // << glm::length(ROPE.getObject(0).f_angularVelocity) << "; " << std::endl;

            
            // std::cout << "    <^> Angular momentum:   ("
		    // << ROPE.getObject(0).f_angularMomentum[0] << "; "
		    // << ROPE.getObject(0).f_angularMomentum[1] << "; "
		    // << ROPE.getObject(0).f_angularMomentum[2] << ")" << std::endl
            // << "    <^> Angular momentum length:   "
            // << glm::length(ROPE.getObject(0).f_angularMomentum) << "; " << std::endl << std::endl;

            std::cout << "    <^> Kinetic energy:"
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
        renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
        forceCopy.onDraw(renderer);
        ROPE.onDraw(renderer);
    }

    void onGUI() override {
        ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1f);
        ImGui::Checkbox("Pause", &this->pause);
    }
};