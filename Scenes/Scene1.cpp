#include "Scene1.h"
#include <iostream>

const float DEFAULT_POINT_MASS = 10.0f;
const glm::vec3 INITIAL_P1_POSITION{ 0.0f,  0.0f,  0.0f};
const glm::vec3 INITIAL_P1_VELOCITY{-1.0f,  0.0f,  0.0f};
const glm::vec3 INITIAL_P2_POSITION{ 0.0f,  2.0f,  0.0f};
const glm::vec3 INITIAL_P2_VELOCITY{ 1.0f,  0.0f,  0.0f};

const float SPRING_STIFFNESS = 40.0f;
const float SPRING_INITIAL_LENGTH = 1.0f;

const float STEP = 0.1f;

void Scene1::init() {
    PointRegistry* point_registry = PointRegistry::getInstance();
    ForceRegistry* force_registry = ForceRegistry::getInstance();

    // Creating Points
    Point p1{DEFAULT_POINT_MASS};
    point_registry->add(p1);

    Point p2{DEFAULT_POINT_MASS};
    point_registry->add(p2);

    // Creating Force Generators
    SpringForceGenerator sfg_p1{p1, SPRING_STIFFNESS, SPRING_INITIAL_LENGTH};
    force_registry->add(p2, sfg_p1);

    SpringForceGenerator sfg_p2{p2, SPRING_STIFFNESS, SPRING_INITIAL_LENGTH};
    force_registry->add(p1, sfg_p2);

    // Set the initial position and velocity
    p1.setPosition(INITIAL_P1_POSITION);
    p1.setVelocity(INITIAL_P1_VELOCITY);
    p2.setPosition(INITIAL_P2_POSITION);
    p2.setVelocity(INITIAL_P2_VELOCITY);
    
    point_registry->setIntegrationMode(PointRegistry::IntegrationMode::EULER);
    point_registry->simulateStep(STEP);

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

    point_registry->setIntegrationMode(PointRegistry::IntegrationMode::MIDPOINT);
    point_registry->simulateStep(STEP);

    std::cout << "------- MIDPOINT STEP RESULT -------" << std::endl;
    std::cout << "POINT 1:" << std::endl;
    std::cout << p1 << std::endl;
    std::cout << "POINT 2:" << std::endl;
    std::cout << p2 << std::endl;
}
