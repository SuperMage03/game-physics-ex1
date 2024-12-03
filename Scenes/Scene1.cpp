#include "Scene1.h"
#include <iostream>
#include "DynamicWorld.h"
#include "ForceRegistry.h"

void Scene1::init() {
    DynamicWorld::getInstance()->setIntegrationMode(DynamicWorld::IntegrationMode::MIDPOINT);
    DynamicWorld::getInstance()->add(_test_cube);
    ForceRegistry::getInstance()->add(_test_cube, _test_torque);
    DynamicWorld::getInstance()->initRigidBodyData();

    _test_cube.setAngularVelocity(glm::vec3(0.0f, 1.0f, 0.2f));
}

void Scene1::simulateStep() {
    DynamicWorld::getInstance()->simulateStep();
}

void Scene1::onDraw(Renderer &renderer) {
    renderer.drawCube(_test_cube.getPosition(), _test_cube.getOrientation(), _test_cube.getScale(), glm::vec4(1.0f));
    // std::cout << _test_cube.getPosition().x << ", " << _test_cube.getPosition().y << ", " << _test_cube.getPosition().z << std::endl;
}

Scene1::~Scene1() {
    DynamicWorld::getInstance()->clear();
    ForceRegistry::getInstance()->clear();
}
