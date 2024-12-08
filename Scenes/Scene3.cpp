#include "Scene3.h"
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
    renderer.drawLine(rb.getPosition(), rb.scaledLocalToWorldPosition(glm::vec3{1.0f, 0.0f, 0.0f}), glm::vec4{1.0f, 0.0f, 0.0f, 1.0f});
    renderer.drawLine(rb.getPosition(), rb.scaledLocalToWorldPosition(glm::vec3{0.0f, 1.0f, 0.0f}), glm::vec4{0.0f, 1.0f, 0.0f, 1.0f});
    renderer.drawLine(rb.getPosition(), rb.scaledLocalToWorldPosition(glm::vec3{0.0f, 0.0f, 1.0f}), glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
}

void Scene3::init() {
    DynamicWorld::getInstance()->addCollisionSolver(_impulse_solver);
    DynamicWorld::getInstance()->addObject(_test_cube_a);
    DynamicWorld::getInstance()->addObject(_test_cube_b);
    _test_cube_a.setLinearVelocity(glm::vec3(0.3f, 0.0f, 0.0f));
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
}

void Scene3::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(_step);
}

void Scene3::onDraw(Renderer &renderer) {
    renderer.drawCube(_test_cube_a.getPosition(), _test_cube_a.getOrientation(), _test_cube_a.getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawCube(_test_cube_b.getPosition(), _test_cube_b.getOrientation(), _test_cube_b.getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
}

void Scene3::onGUI() {
    ImGui::SliderFloat("Time Step", &_step, 0.001f, 2.0f);
}

Scene3::~Scene3() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
