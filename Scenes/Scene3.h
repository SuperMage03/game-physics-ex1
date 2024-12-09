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
    float _step = 0.01f;
    CuboidRigidBody _test_wall_a{glm::vec3(2.0f, 0.0f, 0.0f), glm::vec3(1.0f, 2.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 999.0f, 1.0f, 0.0f};
    CuboidRigidBody _test_wall_b{glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(1.0f, 2.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 999.0f, 1.0f, 0.0f};
    // CuboidRigidBody _test_cube_a{glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    // CuboidRigidBody _test_cube_b{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    CuboidRigidBody _test_cube_a{glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f), glm::quat(0.4619398f, 0.1913417f, 0.4619398f, 0.7325378f), 1.0f, 1.0f, 0.0f};
    // CuboidRigidBody _test_cube_b{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0.5f, 0.0f};
    ImpulseSolver _impulse_solver;
};
