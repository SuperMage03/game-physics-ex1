#include "Transform.h"

glm::vec3 Transform::getPosition() const {
    return _position;
}

void Transform::setPosition(const glm::vec3 &position) {
    _position = position;
}

glm::quat Transform::getOrientation() const {
    return _orientation;
}

void Transform::setOrientation(const glm::quat &orientation) {
    _orientation = orientation;
}

glm::vec3 Transform::getScale() const {
    return _scale;
}

void Transform::setScale(const glm::vec3 &scale) {
    _scale = scale;
}

glm::mat4 Transform::getLocalToWorldMatrix() const {
    glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), _position);
    glm::mat4 rotation_matrix = glm::toMat4(_orientation);
    glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), _scale);
    return translation_matrix * rotation_matrix * scale_matrix;
}

glm::mat4 Transform::getWorldToLocalMatrix() const {
    glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), -_position);
    glm::mat4 rotation_matrix = glm::toMat4(glm::inverse(_orientation));
    glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f / _scale.x, 1.0f / _scale.y, 1.0f / _scale.z));
    return scale_matrix * rotation_matrix * translation_matrix;
}

glm::vec3 Transform::getWorldPosition(const glm::vec3 &local_position) {
    glm::vec4 result = getLocalToWorldMatrix() * glm::vec4(local_position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

glm::vec3 Transform::getLocalPosition(const glm::vec3 &world_position) {
    glm::vec4 result = getWorldToLocalMatrix() * glm::vec4(world_position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}
