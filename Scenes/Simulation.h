#include "Scene.h"
#include <random>

class Simulation : public Scene
{
    virtual void onGUI() override;
    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
    void initializeRigidBodies();
    void applyForceToBody(int rigidBodyIndex, glm::vec3 newForce, glm::vec3 forcePosition);
    void integrationStep(float timeStep);

    struct Force {
        glm::vec3 fiForce;
        glm::vec3 xiPoint;
    };

    struct Rigidbody {
        glm::vec3 XCMpositionCentreOfMass;
        float xWidth;
        float yDepth;
        float zHeight;

        float MtotalMass;

        glm::vec3 vLinearVelocity;

        glm::quat rRotationQuaternion;
        glm::vec3 LangularMomentum;
        glm::vec3 wAngularVelocity;

        std::vector<Force> externalForces;

        glm::mat3x3 I0initialInvertedIntertiaTensor;
    };

    float sizeOfTimeStep = 0.01f;
    float powerOfForce = 5.0f;
    std::vector<Rigidbody> rigidbodies;

    // for the random color of the cubes
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    public:
        Simulation() : gen(rd()), dis(0.f, 1.f) {}
};