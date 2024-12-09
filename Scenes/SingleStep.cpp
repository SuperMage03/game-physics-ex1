#include "SingleStep.h"

void SingleStep::init() {
    // In the <SCENE_NAME>::init() method, use your rigidbody simulation functionality to compute the
    // result for the integration of one time step with ∆t=2 and print it 
    // (updated rigidbody state and the worldspace position and velocity of the aforementioned point on the rigidbody)
    // to the terminal

    // create rigidbodies
    glm::vec3 XCM = glm::vec3(0.0f,0.0f,0.0f);
    float x = 1.0f;
    float y = 0.6f;
    float z = 0.5f;
    float M = 2.0f;
    glm::quat r = glm::normalize(glm::quat(0.7071067812f, 0.0f, 0.0f, 0.7071067812f));
    glm::vec3 v = glm::vec3(0.0f,0.0f,0.0f);
    glm::vec3 w = glm::vec3(0.0f,0.0f,0.0f);
    Rigidbody& testRigidbody = Rigidbody{XCM, x, y, z, M, v, r, glm::vec3(0.0f), w, std::vector<Force>(), glm::mat3x3(1.0f)};
    rigidbodies.push_back(testRigidbody);

    // initialize rigid body
    initializeRigidBodies();
    // apply force
    applyForceToBody(0, glm::vec3(1.0f,1.0f,0.0f), glm::vec3(0.3f, 0.5f, 0.25f));

    // print to console
    std::cout << "___________ Initial state of rigidbody: ____________" << std::endl;
    printStateOfRigidbody(0);
    std::cout << "" << std::endl;

    std::cout << "___________ During integration step: ____________" << std::endl;
    // integrate rigid body setup
    integrationStep(2.0f);
    std::cout << "" << std::endl;

    // print to console
    std::cout << "___________ Updated state of rigidbody: ____________" << std::endl;
    printStateOfRigidbody(0);
    std::cout << "" << std::endl;

    // apply to given point
    PosAndVel& result = updateWorldPosAndVelocity(0, glm::vec3(-0.3f, -0.5f, -0.25f));

    // print to console
    std::cout << "___________ Updated state of given point: ____________" << std::endl;
    std::cout << "Updated position: xi = (" << result.position.x << ", " << result.position.y << ", " << result.position.z << ")" << std::endl;
    std::cout << "Updated velocity: vi = (" << result.velocity.x << ", " << result.velocity.y << ", " << result.velocity.z << ")" << std::endl;
}

void SingleStep::initializeRigidBodies() {
    for(Rigidbody& body: rigidbodies) {
        // initialize inertia tensor
        glm::mat3x3 Iinverted = glm::mat3x3();
        Iinverted[0][0] = 1.0f/((1.0f/12.0f) * body.MtotalMass * (glm::pow(body.zHeight, 2.0f) + glm::pow(body.yDepth, 2.0f)));
        Iinverted[1][1] = 1.0f/((1.0f/12.0f) * body.MtotalMass * (glm::pow(body.xWidth, 2.0f) + glm::pow(body.zHeight, 2.0f)));
        Iinverted[2][2] = 1.0f/((1.0f/12.0f) * body.MtotalMass * (glm::pow(body.xWidth, 2.0f) + glm::pow(body.yDepth, 2.0f)));
        body.I0initialInvertedIntertiaTensor = Iinverted;

        // initialize angular momentum
        glm::vec3 secondOperand = glm::vec3(body.MtotalMass * body.vLinearVelocity.x, body.MtotalMass * body.vLinearVelocity.y, body.MtotalMass * body.vLinearVelocity.z);
        body.LangularMomentum = glm::cross(body.XCMpositionCentreOfMass, secondOperand);

        // initialize angular momentum and angular velocity
        body.LangularMomentum = glm::cross(body.XCMpositionCentreOfMass, body.MtotalMass * body.vLinearVelocity);
        glm::mat3x3 Rotr = glm::mat3_cast(body.rRotationQuaternion);
        glm::mat3x3 IcurrentInvertedIntertiaTensor = Rotr * body.I0initialInvertedIntertiaTensor * glm::transpose(Rotr);
        body.wAngularVelocity = IcurrentInvertedIntertiaTensor * body.LangularMomentum;
    }
}

void SingleStep::printStateOfRigidbody(int rigidBodyIndex) {
    Rigidbody& body = rigidbodies[rigidBodyIndex];
    std::cout << "Centre of mass: Xcm = (" << body.XCMpositionCentreOfMass.x << ", " << body.XCMpositionCentreOfMass.y << ", " << body.XCMpositionCentreOfMass.z << ")" << std::endl;
    std::cout << "Linear velocity: v = (" << body.vLinearVelocity.x << ", " << body.vLinearVelocity.y << ", " << body.vLinearVelocity.z << ")" << std::endl;
    std::cout << "Rotation quaternion: r = (" << body.rRotationQuaternion.w << ", " << body.rRotationQuaternion.x << ", " << body.rRotationQuaternion.y << ", " << body.rRotationQuaternion.z << ")" << std::endl;
    std::cout << "Angular momentum: L = (" << body.LangularMomentum.x << ", " << body.LangularMomentum.y << ", " << body.LangularMomentum.z << ")" << std::endl;
    std::cout << "Angular velocity: w = (" << body.wAngularVelocity.x << ", " << body.wAngularVelocity.y << ", " << body.wAngularVelocity.z << ")" << std::endl;
}

void SingleStep::applyForceToBody(int rigidBodyIndex, glm::vec3 newForce, glm::vec3 forcePosition) {
    rigidbodies[rigidBodyIndex].externalForces.push_back(Force{newForce, forcePosition});
}

void SingleStep::integrationStep(float timeStep) {

    // perform step for every rigidbody
    for(Rigidbody& body: rigidbodies) {

        // compute total external force F and torque q
        glm::vec3 FtotalExternalForce = glm::vec3(0.0f);
        glm::vec3 qTorque = glm::vec3(0.0f);
        for(Force force: body.externalForces) {
            FtotalExternalForce += force.fiForce;
            qTorque += glm::cross(force.xiPoint, force.fiForce);
        }
        std::cout << "Torque: q = (" << qTorque.x << ", " << qTorque.y << ", " << qTorque.z << ")" << std::endl;
        std::cout << "Total external force: F = (" << FtotalExternalForce.x << ", " << FtotalExternalForce.y << ", " << FtotalExternalForce.z << ")" << std::endl;

        // Euler step
        body.XCMpositionCentreOfMass = body.XCMpositionCentreOfMass + timeStep * body.vLinearVelocity;
        body.vLinearVelocity = body.vLinearVelocity + timeStep * (FtotalExternalForce / body.MtotalMass);

        body.rRotationQuaternion = glm::normalize(body.rRotationQuaternion + (timeStep / 2.0f) * glm::quat(0.0f, body.wAngularVelocity) * body.rRotationQuaternion);

        body.LangularMomentum = body.LangularMomentum + timeStep * qTorque;

        glm::mat3x3 Rotr = glm::mat3_cast(body.rRotationQuaternion);
        glm::mat3x3 IcurrentInvertedIntertiaTensor = Rotr * body.I0initialInvertedIntertiaTensor * glm::transpose(Rotr);
        
        body.wAngularVelocity = IcurrentInvertedIntertiaTensor * body.LangularMomentum;

        // clear external forces
        body.externalForces.clear();
    }
}

SingleStep::PosAndVel SingleStep::updateWorldPosAndVelocity(int rigidBodyIndex, glm::vec3 point) {
    Rigidbody& body = rigidbodies[rigidBodyIndex];
    glm::mat3x3 Rotr = glm::mat3_cast(body.rRotationQuaternion);
    glm::vec3 pos = body.XCMpositionCentreOfMass + Rotr * point;
    glm::vec3 vel = body.vLinearVelocity + glm::cross(body.wAngularVelocity, point);
    return PosAndVel{pos, vel};
}