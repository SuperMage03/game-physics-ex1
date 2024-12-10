#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ForceGenerator.hpp"
#include "CuboidRigidBody.hpp"
#include "CollisionSolver.hpp"

class Scene3 : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    void onGUI() override;
    ~Scene3() override;
private:
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);
    ExternalForceGenerator _interaction_external_force{glm::vec3(0.0f), glm::vec3(0.0f)};

    float _step = 0.01f;

    CuboidRigidBody _test_cube_a{glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(1.0f), glm::quat(0.4619398f, 0.1913417f, 0.4619398f, 0.7325378f), 1.0f, 0.5f, 0.0f};
    CuboidRigidBody _test_cube_b{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0.5f, 0.0f};
    ImpulseSolver _impulse_solver;
};
