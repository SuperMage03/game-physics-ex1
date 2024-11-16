#pragma once
#include <glm/glm.hpp>

const float DEFAULT_POINT_MASS = 10.0f;
const glm::vec3 INITIAL_P1_POSITION{ 0.0f,  0.0f,  0.0f};
const glm::vec3 INITIAL_P1_VELOCITY{-1.0f,  0.0f,  0.0f};
const glm::vec3 INITIAL_P2_POSITION{ 0.0f,  2.0f,  0.0f};
const glm::vec3 INITIAL_P2_VELOCITY{ 1.0f,  0.0f,  0.0f};

const float SPRING_STIFFNESS = 40.0f;
const float SPRING_INITIAL_LENGTH = 1.0f;
