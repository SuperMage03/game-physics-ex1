#include "Scene.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "ForceGenerator.hpp"
#include "CuboidRigidBody.hpp"

class Scene2 : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    void onGUI() override;
    ~Scene2() override;
private:
    glm::mat4 cameraMatrix = glm::mat4(1);
    glm::vec3 fwd = glm::vec3(1, 0, 0);
    glm::vec3 right = glm::vec3(0, 1, 0);
    glm::vec3 up = glm::vec3(0, 0, 1);
    ExternalForceGenerator _interaction_external_force{glm::vec3(0.0f), glm::vec3(0.0f)};

    float _step = 0.01f;

    CuboidRigidBody _test_cube{glm::vec3(0.0f), glm::vec3(1.0f, 0.6f, 0.5f), glm::quat(0.7071068f, 0.0f, 0.0f, 0.7071068f), 2.0f};
    ExternalForceGenerator _test_external_force{glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.3f, 0.5f, 0.25f)};
};
