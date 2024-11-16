#pragma once
#include "Renderer.h"

class Environment {
private:
    glm::vec3 _gravity;

public:
    Environment():
    _gravity(glm::vec3(0.f, 0.f, -9.81f))
    {}

    glm::vec3 getGravity() const;

    void setGravity(glm::vec3 gravity);
};

