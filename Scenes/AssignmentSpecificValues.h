#pragma once
#include <glm/glm.hpp>

const float DEFAULT_POINT_MASS = 10.0f;
const glm::vec3 INITIAL_P1_POSITION{ 0.0f,  0.0f,  0.0f};
const glm::vec3 INITIAL_P1_VELOCITY{-1.0f,  0.0f,  0.0f};
const glm::vec3 INITIAL_P2_POSITION{ 0.0f,  2.0f,  0.0f};
const glm::vec3 INITIAL_P2_VELOCITY{ 1.0f,  0.0f,  0.0f};

const float SPRING_STIFFNESS = 40.0f;
const float SPRING_INITIAL_LENGTH = 1.0f;

const float PHI = 1.61803398875f;

const int ICOSAHEDRON_VERTEX_COUNT = 12;

const glm::vec3 ICOSAHEDRON_VERTICES[ICOSAHEDRON_VERTEX_COUNT] = {
    { PHI,  1.0f, 0.0f},
    { PHI, -1.0f, 0.0f},
    {-PHI, -1.0f, 0.0f},
    {-PHI,  1.0f, 0.0f},

    { 1.0f,  0.0f,  PHI},
    {-1.0f,  0.0f,  PHI},
    {-1.0f,  0.0f, -PHI},
    { 1.0f,  0.0f, -PHI},

    { 0.0f,  PHI,  1.0f},
    { 0.0f,  PHI, -1.0f},
    { 0.0f, -PHI, -1.0f},
    { 0.0f, -PHI,  1.0f},
};

const int ICOSAHEDRON_EDGE_COUNT = 30;

const std::pair<int, int> ICOSAHEDRON_EDGES[ICOSAHEDRON_EDGE_COUNT] = {
    { 0, 1 }, { 0, 4 }, { 0, 7 }, { 0, 8 }, { 0, 9 }, { 1, 4 }, 
    { 1, 7 }, { 1, 10}, { 1, 11}, { 2, 3 }, { 2, 5 }, { 2, 6 },
    { 2, 10}, { 2, 11}, { 3, 5 }, { 3, 6 }, { 3, 8 }, { 3, 9 },
    { 4, 5 }, { 4, 8 }, { 4, 11}, { 5, 8 }, { 5, 11}, { 6, 7 },
    { 6, 9 }, { 6, 10}, { 7, 9 }, { 7, 10}, { 8, 9 }, {10, 11},
};

const glm::vec3 BOUNDING_BOX_POSITION = {0.0f, 0.0f, 0.0f};
const glm::vec3 BOUNDING_BOX_SCALE = {5.0f, 5.0f, 5.0f};
