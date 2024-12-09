#include "Scene4.h"
#include <imgui.h>
#include "DynamicWorld.h"
#include "ForceRegistry.h"

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
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::MIDPOINT);
    DynamicWorld::getInstance()->addCollisionSolver(_impulse_solver);
    DynamicWorld::getInstance()->addObject(_ground_cube);
    DynamicWorld::getInstance()->addObject(_test_cube_a);
    DynamicWorld::getInstance()->addObject(_test_cube_b);
    // _test_cube_a.setLinearVelocity(glm::vec3(0.5f, 0.0f, 0.0f));
    // _test_cube_b.setLinearVelocity(glm::vec3(-0.2f, 0.0f, 0.0f));
    ForceRegistry::getInstance()->add(_test_cube_a, gravity_force_generator);
    ForceRegistry::getInstance()->add(_test_cube_b, gravity_force_generator);
}

void Scene4::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(_step);
}

void Scene4::onDraw(Renderer &renderer) {
    renderer.drawCube(_test_cube_a.getTransform().getPosition(), _test_cube_a.getTransform().getOrientation(), _test_cube_a.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawCube(_test_cube_b.getTransform().getPosition(), _test_cube_b.getTransform().getOrientation(), _test_cube_b.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawCube(_ground_cube.getTransform().getPosition(), _ground_cube.getTransform().getOrientation(), _ground_cube.getTransform().getScale(), glm::vec4(1.0f));
}

void Scene4::onGUI() {
    ImGui::SliderFloat("Time Step", &_step, 0.001f, 2.0f);
}

Scene4::~Scene4() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
