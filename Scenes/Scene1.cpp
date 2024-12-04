#include "Scene1.h"
#include <iostream>
#include "DynamicWorld.h"
#include "ForceRegistry.h"

// File scope helper function for drawing the axis along the origin
static void drawAxis(Renderer& renderer) {
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{1.0f, 0.0f, 0.0f}, glm::vec4{1.0f, 0.0f, 0.0f, 1.0f});
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{0.0f, 1.0f, 0.0f}, glm::vec4{0.0f, 1.0f, 0.0f, 1.0f});
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{0.0f, 0.0f, 1.0f}, glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
}

void Scene1::init() {
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::EULER);
    DynamicWorld::getInstance()->add(_test_cube);
    ForceRegistry::getInstance()->add(_test_cube, _test_torque);

    _test_cube.setAngularVelocity(glm::vec3(0.0f, 0.1f, 0.000001f));
}

void Scene1::simulateStep() {
    DynamicWorld::getInstance()->simulateStep(0.1f);
}

void Scene1::onDraw(Renderer &renderer) {
    renderer.drawCube(_test_cube.getPosition(), _test_cube.getOrientation(), _test_cube.getScale(), glm::vec4(1.0f, 1.0f, 1.0f, 0.5f));
    renderer.drawLine(_test_cube.getPosition(), _test_cube.getPosition() + _test_cube.getAngularVelocity(), glm::vec4(1.0f, 0.5f, 0.0f, 1.0f));
    // std::cout << _test_cube.getPosition().x << ", " << _test_cube.getPosition().y << ", " << _test_cube.getPosition().z << std::endl;
    drawAxis(renderer);
}

Scene1::~Scene1() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
