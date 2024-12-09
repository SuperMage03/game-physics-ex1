#include "Collision.h"
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>
#include "util/CollisionDetection.h"

void Collision::init() {
    // create rigidbodies
    // left one
    glm::vec3 XCM = glm::vec3(0.0f,-1.0f,0.0f);
    float x = 0.5f;
    float y = 0.5f;
    float z = 0.5f;
    float M = 2.0f;
    glm::quat r = glm::normalize(glm::quat(0.0f, 0.0f, 0.0f, 0.0f));
    glm::vec3 v = glm::vec3(0.0f,0.0f,0.0f);
    glm::vec3 w = glm::vec3(0.0f,0.0f,0.0f);
    Rigidbody& testRigidbody = Rigidbody{XCM, x, y, z, M, v, r, glm::vec3(0.0f), w, std::vector<Force>(), glm::mat3x3(1.0f)};
    rigidbodies.push_back(testRigidbody);
    // right one
    glm::vec3 XCMr = glm::vec3(0.0f,1.0f,0.0f);
    float xr = 0.5f;
    float yr = 0.5f;
    float zr = 0.5f;
    float Mr = 2.0f;
    glm::quat rr = glm::normalize(glm::quat(-0.6532815f, -0.2705981f, -0.6532815f, 0.7071067812f));
    glm::vec3 vr = glm::vec3(0.0f,-0.25f,0.0f);
    glm::vec3 wr = glm::vec3(0.0f,0.0f,0.0f);
    Rigidbody& testRigidbodyr = Rigidbody{XCMr, xr, yr, zr, Mr, vr, rr, glm::vec3(0.0f), wr, std::vector<Force>(), glm::mat3x3(1.0f)};
    rigidbodies.push_back(testRigidbodyr);

    // initialize rigid bodies
    initializeRigidBodies();
    // apply force
    //applyForceToBody(1, glm::vec3(0.0f,-1.0f,0.0f), glm::vec3(0.0f,1.0f,0.0f));
}

void Collision::onDraw(Renderer& renderer) {
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

void Collision::onGUI(){
    ImGui::SliderFloat("Size of time step", &sizeOfTimeStep, 0.001f, 0.2f);

    if(ImGui::Button("Do time step (or press space)") || ImGui::IsKeyDown(ImGuiKey_Space)) {
        integrationStep(sizeOfTimeStep);
        handleCollisions();
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
            handleCollisions();
            std::cout << "Applied force to body " << i << std::endl;
        }

        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
    }
}

void Collision::initializeRigidBodies() {
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

void Collision::applyForceToBody(int rigidBodyIndex, glm::vec3 newForce, glm::vec3 forcePosition) {
    rigidbodies[rigidBodyIndex].externalForces.push_back(Force{newForce, forcePosition});
}

void Collision::integrationStep(float timeStep) {

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

void Collision::handleCollisions() {
    // check collision of every body with every other body only once
    for(int i = 1; i < rigidbodies.size(); i++) {
        for(int j = 0; j < i; j++) {
            Rigidbody& a = rigidbodies[i];
            Rigidbody& b = rigidbodies[j];
            glm::mat4 matrixA = getWorldFromObj(a);
            glm::mat4 matrixB = getWorldFromObj(b);
            CollisionInfo info = collisionTools::checkCollisionSAT(matrixA, matrixB);

            // there is a collision, handle it
            if(info.isColliding) {
                // Calculate the collision point’s world position, the collision face’s normal, 
                // the velocity difference between the two boxes at the collision point in world space, 
                // and the impulse J. 
                // Update the velocity and momentum of the two boxes according to J as described in the lecture slides.

                // Based on the returned data, implement your collision response method by calculating the impulse and updating the momentums.

                // Once you find a collision, your collision response function should calculate:
                // • vrel, the relative velocity between A and B at the collision point (in world space).
                // • If vrel · n > 0, this indicates that the bodies are separating.
                // • Otherwise continue to calculate the impulse J, and apply it to both bodies.

                // calculate v rel
                glm::vec3 cCollisionPointWorld = info.collisionPointWorld;
                glm::vec3 distanceAtoC = cCollisionPointWorld - a.XCMpositionCentreOfMass;
                glm::vec3 distanceBtoC = cCollisionPointWorld - b.XCMpositionCentreOfMass;
                glm::vec3 velocityDifferenceA = a.vLinearVelocity + glm::cross(distanceAtoC, a.wAngularVelocity);
                glm::vec3 velocityDifferenceB = b.vLinearVelocity + glm::cross(distanceBtoC, b.wAngularVelocity);
                glm::vec3 vRel = velocityDifferenceA - velocityDifferenceB;

                // check if bodies are separating
                glm::vec3 nNormalVector = info.normalWorld;
                // bodies are not separating
                if(glm::dot(vRel, nNormalVector) <= 0) {
                    // calculate impulse J
                    float c = 1.0f;
                    // compute current inertia tensors
                    glm::mat3x3 RotrA = glm::mat3_cast(a.rRotationQuaternion);
                    glm::mat3x3 IcurrentInvertedIntertiaTensorA = RotrA * a.I0initialInvertedIntertiaTensor * glm::transpose(RotrA);
                    glm::mat3x3 RotrB = glm::mat3_cast(b.rRotationQuaternion);
                    glm::mat3x3 IcurrentInvertedIntertiaTensorB = RotrB * b.I0initialInvertedIntertiaTensor * glm::transpose(RotrB);
                    // the upper part of the term to calculate J
                    float top = -(1.0f + c) * glm::dot(vRel, nNormalVector);
                    float bottom = (1.0f / a.MtotalMass) + (1.0f / b.MtotalMass) 
                        + dot((cross(IcurrentInvertedIntertiaTensorA * glm::cross(distanceAtoC, nNormalVector), distanceAtoC)), nNormalVector)
                        + dot((cross(IcurrentInvertedIntertiaTensorB * glm::cross(distanceBtoC, nNormalVector), distanceBtoC)), nNormalVector);
                    float Jimpulse = top / bottom;

                    // update velocities
                    a.vLinearVelocity = a.vLinearVelocity + Jimpulse * nNormalVector / a.MtotalMass;
                    b.vLinearVelocity = b.vLinearVelocity - Jimpulse * nNormalVector / b.MtotalMass;

                    // update momentums
                    glm::vec3 La = a.LangularMomentum + glm::cross(distanceAtoC, Jimpulse * nNormalVector);
                    glm::vec3 Lb = b.LangularMomentum - glm::cross(distanceBtoC, Jimpulse * nNormalVector);
                    a.LangularMomentum = La;
                    b.LangularMomentum = Lb;

                    a.wAngularVelocity = IcurrentInvertedIntertiaTensorA * a.LangularMomentum;
                    b.wAngularVelocity = IcurrentInvertedIntertiaTensorB * b.LangularMomentum;
                } 
            }
        }
    }
}

glm::mat4 Collision::getWorldFromObj(Rigidbody rb) {
    glm::mat4 rotationMatrix = glm::toMat4(rb.rRotationQuaternion);
    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1), glm::vec3(rb.xWidth, rb.yDepth, rb.zHeight));
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1), rb.XCMpositionCentreOfMass);
    return translationMatrix * rotationMatrix * scaleMatrix;
}