#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ForceGenerator.h"
#include "CuboidRigidBody.h"
#include "CollisionSolver.h"

class Scene3 : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    void onGUI() override;
    ~Scene3() override;
private:
    float _step = 0.01f;
    CuboidRigidBody _test_cube_a{glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    CuboidRigidBody _test_cube_b{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    ImpulseSolver _impulse_solver;
};
