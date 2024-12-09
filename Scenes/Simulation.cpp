#include "Simulation.h"
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>

void Simulation::init() {
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
}

void Simulation::onDraw(Renderer& renderer) {
    for (Rigidbody& body : rigidbodies) {
        renderer.drawCube(body.XCMpositionCentreOfMass, body.rRotationQuaternion,  glm::vec3(body.xWidth, body.yDepth, body.zHeight), glm::vec4(dis(gen), dis(gen), dis(gen), 1));
    }

    // copied from assignment zero for displaying the coordinate system                                       
    glm::vec3 forward = glm::vec3(glm::vec4(0, 0, 1, 0));
    glm::vec3 right = glm::vec3(glm::vec4(1, 0, 0, 0));
    glm::vec3 up = glm::vec3(glm::vec4(0, 1, 0, 0));

    renderer.drawLine(glm::vec3(0), forward, glm::vec4(1, 0, 0, 1));
    renderer.drawLine(glm::vec3(0), right, glm::vec4(0, 1, 0, 1));
    renderer.drawLine(glm::vec3(0), up, glm::vec4(0, 0, 1, 1));
}

void Simulation::onGUI(){
    ImGui::SliderFloat("Size of time step", &sizeOfTimeStep, 0.001f, 0.2f);

    if(ImGui::Button("Do time step (or press space)") || ImGui::IsKeyDown(ImGuiKey_Space)) {
        integrationStep(sizeOfTimeStep);
    }

    // for applying forces through right click and drag
    ImGui::Text("Apply force through right click and drag");
    ImGui::SliderFloat("Power of force", &powerOfForce, 0.001f, 10.0f);
    glm::vec2 screenSize = glm::vec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y);
    // when mouse is pressed
    glm::vec3 positionOfForce = glm::vec3(0.0f);
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        ImVec2 mouseClick = ImGui::GetMousePos();
        glm::vec2 mouseClickNormalized = glm::vec2(mouseClick.x, mouseClick.y) / screenSize;
        positionOfForce = glm::vec3(0.0f, mouseClickNormalized.x, mouseClickNormalized.y);
    }
    // when mouse is released
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
        ImVec2 mouseDrag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
        // normalize to value between 0 and 1, depending on size of screen
        glm::vec2 mouseDragNormalized = glm::vec2(mouseDrag.x, mouseDrag.y) / screenSize;
        glm::vec3 forceToApply = glm::vec3(0.0f, mouseDragNormalized.x * powerOfForce, mouseDragNormalized.y * powerOfForce * (-1.0f));
        std::cout << "ForceToApply: f = (" << forceToApply.x << ", " << forceToApply.y << ", " << forceToApply.z << ")" << std::endl;

        // apply force to all rigidbodies
        for(int i = 0; i < rigidbodies.size(); i++) {
            applyForceToBody(i, forceToApply, positionOfForce);
            integrationStep(sizeOfTimeStep);
            std::cout << "Applied force to body " << i << std::endl;
        }

        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
    }
}

void Simulation::initializeRigidBodies() {
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

void Simulation::applyForceToBody(int rigidBodyIndex, glm::vec3 newForce, glm::vec3 forcePosition) {
    rigidbodies[rigidBodyIndex].externalForces.push_back(Force{newForce, forcePosition});
}

void Simulation::integrationStep(float timeStep) {

    // perform step for every rigidbody
    for(Rigidbody& body: rigidbodies) {

        // compute total external force F and torque q
        glm::vec3 FtotalExternalForce = glm::vec3(0.0f);
        glm::vec3 qTorque = glm::vec3(0.0f);
        for(Force force: body.externalForces) {
            FtotalExternalForce += force.fiForce;
            qTorque += glm::cross(force.xiPoint, force.fiForce);
        }                                                                            

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