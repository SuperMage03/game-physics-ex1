#include "Transform.h"

Transform::Transform(const glm::vec3 &positon, const glm::quat &orientation, const glm::vec3 &scale): 
    _position{positon}, _orientation{orientation}, _scale{scale} {}

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
    _orientation = glm::normalize(orientation);
}

glm::vec3 Transform::getScale() const {
    return _scale;
}

void Transform::setScale(const glm::vec3 &scale) {
    _scale = scale;
}

glm::mat4 Transform::getUnscaledLocalToWorldMatrix() const {
    glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), _position);
    glm::mat4 rotation_matrix = glm::toMat4(_orientation);
    return translation_matrix * rotation_matrix;
}

glm::mat4 Transform::getWorldToUnscaledLocalMatrix() const {
    glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), -_position);
    glm::mat4 rotation_matrix = glm::toMat4(glm::inverse(_orientation));
    return rotation_matrix * translation_matrix;
}

glm::vec3 Transform::getUnscaledLocalToWorldPosition(const glm::vec3 &position) const {
    glm::vec4 result = getUnscaledLocalToWorldMatrix() * glm::vec4(position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

glm::vec3 Transform::getWorldToUnscaledLocalPosition(const glm::vec3 &position) const {
    glm::vec4 result = getWorldToUnscaledLocalMatrix() * glm::vec4(position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

glm::mat4 Transform::getScaledLocalToWorldMatrix() const {
    glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), _scale);
    return getUnscaledLocalToWorldMatrix() * scale_matrix;
}

glm::mat4 Transform::getWorldToScaledLocalMatrix() const {
    glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f / _scale.x, 1.0f / _scale.y, 1.0f / _scale.z));
    return scale_matrix * getWorldToUnscaledLocalMatrix();
}

glm::vec3 Transform::getScaledLocalToWorldPosition(const glm::vec3 &position) const {
    glm::vec4 result = getScaledLocalToWorldMatrix() * glm::vec4(position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

glm::vec3 Transform::getWorldToScaledLocalPosition(const glm::vec3 &position) const {
    glm::vec4 result = getWorldToScaledLocalMatrix() * glm::vec4(position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

std::ostream &operator<<(std::ostream &out_stream, const Transform &transform) {
    out_stream << "<--------- Transform Output --------->" << std::endl;
    out_stream << "Position: " << transform._position.x << ", " << transform._position.y << ", " << transform._position.z << std::endl;
    out_stream << "Orientation: " << transform._orientation.w << ", " << transform._orientation.x << ", " << transform._orientation.y << ", " << transform._orientation.z << std::endl;
    out_stream << "Scale: " << transform._scale.x << ", " << transform._scale.y << ", " << transform._scale.z << std::endl;
    return out_stream;
}
