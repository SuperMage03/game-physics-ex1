#include "Scene1.h"
#include <iostream>

void Scene1::init() {
    point_registry_ = PointRegistry::getInstance();
    force_registry_ = ForceRegistry::getInstance();

    // Register Points and Force Generators
    point_registry_->add(p1);
    point_registry_->add(p2);
    force_registry_->add(p2, sfg_p1);
    force_registry_->add(p1, sfg_p2);

    // Set the initial position and velocity
    p1.setPosition(INITIAL_P1_POSITION);
    p1.setVelocity(INITIAL_P1_VELOCITY);
    p2.setPosition(INITIAL_P2_POSITION);
    p2.setVelocity(INITIAL_P2_VELOCITY);
    
    point_registry_->setIntegrationMode(PointRegistry::IntegrationMode::EULER);
    point_registry_->simulateStep(step);

    std::cout << "-------- EULER STEP RESULT --------" << std::endl;
    std::cout << "POINT 1:" << std::endl;
    std::cout << p1 << std::endl;
    std::cout << "POINT 2:" << std::endl;
    std::cout << p2 << std::endl;

    // Set the initial position and velocity
    p1.setPosition(INITIAL_P1_POSITION);
    p1.setVelocity(INITIAL_P1_VELOCITY);
    p2.setPosition(INITIAL_P2_POSITION);
    p2.setVelocity(INITIAL_P2_VELOCITY);

    point_registry_->setIntegrationMode(PointRegistry::IntegrationMode::MIDPOINT);
    point_registry_->simulateStep(step);

    std::cout << "------- MIDPOINT STEP RESULT -------" << std::endl;
    std::cout << "POINT 1:" << std::endl;
    std::cout << p1 << std::endl;
    std::cout << "POINT 2:" << std::endl;
    std::cout << p2 << std::endl;
}

Scene1::~Scene1() {
    force_registry_->clear();
    point_registry_->clear();
}
