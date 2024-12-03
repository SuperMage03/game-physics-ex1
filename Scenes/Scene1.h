#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "DynamicWorld.h"
#include "ForceGenerator.h"
#include "CuboidRigidBody.h"

class Scene1 : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    ~Scene1() override;
private:
    CuboidRigidBody _test_cube{100.0f, glm::vec3(0.0f), glm::vec3(10.0f, 5.0f, 1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    GenericTorqueGenerator _test_torque{glm::vec3(0.0f, 0.0f, 0.0f)};
};
