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
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
}

void Scene2::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(_step);
}

void Scene2::onDraw(Renderer &renderer) {
    renderer.drawCube(_test_cube.getTransform().getPosition(), _test_cube.getTransform().getOrientation(), _test_cube.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    // renderer.drawLine(_test_cube.getPosition(), _test_cube.getPosition() + _test_cube.getTorque(), glm::vec4(1.0f, 0.5f, 0.0f, 1.0f));
    // std::cout << _test_cube.getPosition().x << ", " << _test_cube.getPosition().y << ", " << _test_cube.getPosition().z << std::endl;
    // drawWorldAxis(renderer);
    // drawLocalAxis(renderer, _test_cube);
    // renderer.drawSphere(glm::vec3(0.3f, 0.5f, 0.25f), 0.05f);
}

void Scene2::onGUI() {
    ImGui::SliderFloat("Time Step", &_step, 0.001f, 2.0f);
}

Scene2::~Scene2() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
