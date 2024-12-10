#include "Scene4.h"
#include <imgui.h>
#include "DynamicWorld.hpp"
#include "ForceRegistry.hpp"

// File scope helper function for drawing the axis along the origin
static void drawWorldAxis(Renderer& renderer) {
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{1.0f, 0.0f, 0.0f}, glm::vec4{1.0f, 0.0f, 0.0f, 1.0f});
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{0.0f, 1.0f, 0.0f}, glm::vec4{0.0f, 1.0f, 0.0f, 1.0f});
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{0.0f, 0.0f, 1.0f}, glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
}

// File scope helper function for drawing the axis along the origin
static void drawLocalAxis(Renderer& renderer, RigidBody& rb) {
    renderer.drawLine(rb.getTransform().getPosition(), rb.getTransform().getScaledLocalToWorldPosition(glm::vec3{1.0f, 0.0f, 0.0f}), glm::vec4{1.0f, 0.0f, 0.0f, 1.0f});
    renderer.drawLine(rb.getTransform().getPosition(), rb.getTransform().getScaledLocalToWorldPosition(glm::vec3{0.0f, 1.0f, 0.0f}), glm::vec4{0.0f, 1.0f, 0.0f, 1.0f});
    renderer.drawLine(rb.getTransform().getPosition(), rb.getTransform().getScaledLocalToWorldPosition(glm::vec3{0.0f, 0.0f, 1.0f}), glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
}

void Scene4::init() {
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
    DynamicWorld::getInstance()->addCollisionSolver(_impulse_solver);
    DynamicWorld::getInstance()->addWorldBound(_bound);

    DynamicWorld::getInstance()->addCollidableObject(_test_cube_a);
    DynamicWorld::getInstance()->addDynamicObject(_test_cube_a);
    DynamicWorld::getInstance()->addCollidableObject(_test_cube_b);
    DynamicWorld::getInstance()->addDynamicObject(_test_cube_b);
    // _test_cube_a.setLinearVelocity(glm::vec3(0.5f, 0.0f, 0.0f));
    // _test_cube_b.setLinearVelocity(glm::vec3(-0.2f, 0.0f, 0.0f));
    ForceRegistry::getInstance()->add(_test_cube_a, gravity_force_generator);
    ForceRegistry::getInstance()->add(_test_cube_b, gravity_force_generator);
    ForceRegistry::getInstance()->add(_test_cube_a, _interaction_external_force);
}

void Scene4::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(_step);
}

void Scene4::onDraw(Renderer &renderer) {
    cameraMatrix = renderer.camera.viewMatrix;
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);
    
    renderer.drawCube(_test_cube_a.getTransform().getPosition(), _test_cube_a.getTransform().getOrientation(), _test_cube_a.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawCube(_test_cube_b.getTransform().getPosition(), _test_cube_b.getTransform().getOrientation(), _test_cube_b.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawQuad(glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec2(20.0f), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
}

void Scene4::onGUI() {
    ImGui::SliderFloat("Time Step", &_step, 0.001f, 2.0f);

    if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){   
        auto drag = ImGui::GetMouseDragDelta(1);
        if(drag.x != 0 || drag.y != 0) {
            glm::vec3 dx = drag.x * right;
            glm::vec3 dy = -drag.y * up;
            _interaction_external_force.setForce((dx + dy), _test_cube_a.getCenterOfMassWorld());
        }
        else {
            _interaction_external_force.setForce(glm::vec3(0.0f), glm::vec3(0.0f));
        }
    }
    else {
        _interaction_external_force.setForce(glm::vec3(0.0f), glm::vec3(0.0f));
    }
}

Scene4::~Scene4() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
