#include "Scene.h"

class SingleStep : public Scene
{
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

    struct PosAndVel {
        glm::vec3 position;
        glm::vec3 velocity;
    };
    
    virtual void init() override;
    void initializeRigidBodies();
    void printStateOfRigidbody(int rigidBodyIndex);
    void integrationStep(float timeStep);
    void applyForceToBody(int rigidBodyIndex, glm::vec3 newForce, glm::vec3 forcePosition);
    PosAndVel updateWorldPosAndVelocity(int rigidBodyIndex, glm::vec3 point);

    std::vector<Rigidbody> rigidbodies;
};