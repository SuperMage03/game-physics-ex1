#include "Scene3.h"
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

void Scene3::init() {
    DynamicWorld::getInstance()->addCollisionSolver(_impulse_solver);
    DynamicWorld::getInstance()->addCollidableObject(_test_cube_a);
    DynamicWorld::getInstance()->addDynamicObject(_test_cube_a);
    // DynamicWorld::getInstance()->addObject(_test_cube_b);
    DynamicWorld::getInstance()->addCollidableObject(_test_wall_a);
    DynamicWorld::getInstance()->addCollidableObject(_test_wall_b);
    _test_cube_a.setLinearVelocity(glm::vec3(0.8f, 0.0f, 0.0f));
    // _test_cube_b.setLinearVelocity(glm::vec3(-0.2f, 0.0f, 0.0f));
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
}

void Scene3::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(_step);
}

void Scene3::onDraw(Renderer &renderer) {
    renderer.drawCube(_test_cube_a.getTransform().getPosition(), _test_cube_a.getTransform().getOrientation(), _test_cube_a.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    // renderer.drawCube(_test_cube_b.getTransform().getPosition(), _test_cube_b.getTransform().getOrientation(), _test_cube_b.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawCube(_test_wall_a.getTransform().getPosition(), _test_wall_a.getTransform().getOrientation(), _test_wall_a.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawCube(_test_wall_b.getTransform().getPosition(), _test_wall_b.getTransform().getOrientation(), _test_wall_b.getTransform().getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
}

void Scene3::onGUI() {
    ImGui::SliderFloat("Time Step", &_step, 0.001f, 2.0f);
}

Scene3::~Scene3() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
