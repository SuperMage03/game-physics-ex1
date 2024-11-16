#include "Environment.h"

glm::vec3 Environment::getGravity() const {
    return this->_gravity;
}

void Environment::setGravity(glm::vec3 gravity) {
    this->_gravity = gravity;
}