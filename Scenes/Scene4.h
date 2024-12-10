#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "WorldBound.hpp"
#include "ForceGenerator.hpp"
#include "CuboidRigidBody.hpp"
#include "CollisionSolver.hpp"

class Scene4 : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    void onGUI() override;
    ~Scene4() override;
private:
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);

    float _step = 0.01f;
    
    ImpulseSolver _impulse_solver;
    GravityForceGenerator gravity_force_generator{glm::vec3(0.0f, 0.0f, -9.81f)};
    WorldBound _bound{WorldBound::Axis::Z_AXIS, WorldBound::Limit::MIN, 0.0f};
    ExternalForceGenerator _interaction_external_force{glm::vec3(0.0f), glm::vec3(0.0f)};

    // CuboidRigidBody _test_cube_a{glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    // CuboidRigidBody _test_cube_b{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    CuboidRigidBody _test_cube_a{glm::vec3(-1.0f, 0.0f, 2.0f), glm::vec3(1.0f), glm::quat(0.4619398f, 0.1913417f, 0.4619398f, 0.7325378f), 1.0f, 0.5f, 1.0f};
    CuboidRigidBody _test_cube_b{glm::vec3(1.0f, 0.0f, 2.0f), glm::vec3(1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0.5f, 1.0f};
};
