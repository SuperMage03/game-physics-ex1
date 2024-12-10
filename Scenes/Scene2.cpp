#include "Scene2.h"
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

void Scene2::init() {
    DynamicWorld::getInstance()->addDynamicObject(_test_cube);
    ForceRegistry::getInstance()->add(_test_cube, _test_external_force);
    ForceRegistry::getInstance()->add(_test_cube, _interaction_external_force);
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
}

void Scene2::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(_step);
}

void Scene2::onDraw(Renderer &renderer) {
    cameraMatrix = renderer.camera.viewMatrix;
    fwd = inverse(cameraMatrix) * glm::vec4(0, 0, 1, 0);
    right = inverse(cameraMatrix) * glm::vec4(1, 0, 0, 0);
    up = inverse(cameraMatrix) * glm::vec4(0, 1, 0, 0);
    
    renderer.drawCube(_test_cube.getTransform().getPosition(), _test_cube.getTransform().getOrientation(), _test_cube.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawSphere(glm::vec3(1.0f, 1.0f, 0.0f), 0.03f);
    renderer.drawLine(glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f) + glm::vec3(0.3f, 0.5f, 0.25f), glm::vec3(1.0f, 0.0f, 0.0f));
}

void Scene2::onGUI() {
    ImGui::SliderFloat("Time Step", &_step, 0.001f, 2.0f);

    if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){   
        auto drag = ImGui::GetMouseDragDelta(1);
        if(drag.x != 0 || drag.y != 0) {
            glm::vec3 dx = drag.x * right;
            glm::vec3 dy = -drag.y * up;
            _interaction_external_force.setForce((dx + dy), _test_cube.getCenterOfMassWorld());
        }
        else {
            _interaction_external_force.setForce(glm::vec3(0.0f), glm::vec3(0.0f));
        }
    }
    else {
        _interaction_external_force.setForce(glm::vec3(0.0f), glm::vec3(0.0f));
    }
}

Scene2::~Scene2() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
