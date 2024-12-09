#include "Scene1.h"
#include <iostream>
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

void Scene1::init() {
    DynamicWorld::getInstance()->addObject(_test_cube);
    ForceRegistry::getInstance()->add(_test_cube, _test_external_force);

    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
    DynamicWorld::getInstance()->simulateStep(_step);
    glm::vec3 velocity_of_point = _test_cube.getVelocityOfPoint(glm::vec3(-0.3f, -0.5f, -0.25f));

    std::cout << "-------- Test Cube Result --------" << std::endl;
    std::cout << _test_cube;
    std::cout << "Velocity of Point (-0.3, -0.5, -0.25): " << velocity_of_point.x << ", " << velocity_of_point.y << ", " << velocity_of_point.z << std::endl;
}

void Scene1::simulateStep() {
    // DynamicWorld::getInstance()->simulateStep();
}

void Scene1::onDraw(Renderer &renderer) {
    // renderer.drawCube(_test_cube.getPosition(), _test_cube.getOrientation(), _test_cube.getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    // renderer.drawLine(_test_cube.getPosition(), _test_cube.getPosition() + _test_cube.getTorque(), glm::vec4(1.0f, 0.5f, 0.0f, 1.0f));
    // std::cout << _test_cube.getPosition().x << ", " << _test_cube.getPosition().y << ", " << _test_cube.getPosition().z << std::endl;
    // drawWorldAxis(renderer);
    // drawLocalAxis(renderer, _test_cube);
    // renderer.drawSphere(glm::vec3(0.3f, 0.5f, 0.25f), 0.05f);
}

Scene1::~Scene1() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
