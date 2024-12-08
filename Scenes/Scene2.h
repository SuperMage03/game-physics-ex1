#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ForceGenerator.h"
#include "CuboidRigidBody.h"

class Scene2 : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    void onGUI() override;
    ~Scene2() override;
private:
    float _step = 0.01f;
    CuboidRigidBody _test_cube{glm::vec3(0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    ExternalForceGenerator _test_external_force{glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.3f, 0.5f, 0.25f)};
};
