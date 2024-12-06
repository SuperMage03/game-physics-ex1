#include "RigidBody.h"

RigidBody::RigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass): 
    _position{position}, _scale{scale}, _orientation{orientation}, _mass{mass} {
    _calculateRotationMatrix();
}

glm::vec3 RigidBody::getPosition() const {
    return _position;
}

glm::vec3 RigidBody::getScale() const {
    return _scale;
}

void RigidBody::setScale(const glm::vec3 &scale) {
    _scale = scale;
}

void RigidBody::setPosition(const glm::vec3 &position) {
    _position = position;
}

glm::vec3 RigidBody::getLinearVelocity() const {
    return _linear_velocity;
}

void RigidBody::setLinearVelocity(const glm::vec3 &linear_velocity) {
    _linear_velocity = linear_velocity;
}

glm::vec3 RigidBody::getLinearAcceleration() const {
    return _force / _mass;
}

void RigidBody::setLinearAcceleration(const glm::vec3 &linear_acceleration) {
    _force = _mass * linear_acceleration;
}

glm::vec3 RigidBody::getForce() const {
    return _force;
}

void RigidBody::setForce(const glm::vec3 &force) {
    _force = force;
}

glm::quat RigidBody::getOrientation() const {
    return _orientation;
}

void RigidBody::setOrientation(const glm::quat &orientation) {
    _orientation = glm::normalize(orientation);
}

glm::vec3 RigidBody::getAngularVelocity() const {
    return _localToWorldBasisChange(glm::inverse(_inertia_tensor)) * _angular_momentum;
}

glm::quat RigidBody::getAngularVelocityQuat() const {
    return glm::quat(0.0f, getAngularVelocity());
}

void RigidBody::setAngularVelocity(const glm::vec3 &angular_velocity) {
    _angular_momentum = _localToWorldBasisChange(_inertia_tensor) * angular_velocity;
}

glm::vec3 RigidBody::getAngularMomentum() const {
    return _angular_momentum;
}

void RigidBody::setAngularMomentum(const glm::vec3& angular_momentum) {
    _angular_momentum = angular_momentum;
}

glm::vec3 RigidBody::getTorque() const {
    return _torque;
}

void RigidBody::setTorque(const glm::vec3 &torque) {
    _torque = torque;
}

void RigidBody::addForce(const glm::vec3 &force) {
    _force += force;
}

void RigidBody::addTorque(const glm::vec3 &torque) {
    _torque += torque;
}

void RigidBody::calculateDerviedData() {
    _calculateRotationMatrix();
}

glm::vec3 RigidBody::localToWorldPosition(const glm::vec3 &position) const {
    glm::vec4 result = _getLocalToWorldModelMatrix() * glm::vec4(position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

glm::vec3 RigidBody::worldToLocalPosition(const glm::vec3 &position) const {
    glm::vec4 result = _getWorldToLocalModelMatrix() * glm::vec4(position, 1.0f);
    return glm::vec3(result.x, result.y, result.z);
}

glm::vec3 RigidBody::getVelocityOfPoint(const glm::vec3 &point_world_position) const {
    return getLinearVelocity() + _getTangentialVelocityOfPoint(point_world_position);
}

std::ostream &operator<<(std::ostream &out_stream, const RigidBody &rb) {
    out_stream << "Mass Point Position: " << rb._position.x << ", " << rb._position.y << ", " << rb._position.z << std::endl;
    out_stream << "Linear Velocity: " << rb._linear_velocity.x << ", " << rb._linear_velocity.y << ", " << rb._linear_velocity.z << std::endl;
    out_stream << "Force: " << rb._force.x << ", " << rb._force.y << ", " << rb._force.z << std::endl;
    out_stream << "Orientation Quaternion: " << rb._orientation.w << ", " << rb._orientation.x << ", " << rb._orientation.y << ", " << rb._orientation.z << std::endl;
    out_stream << "Angular Velocity: " << rb.getAngularVelocity().x << ", " << rb.getAngularVelocity().y << ", " << rb.getAngularVelocity().z << std::endl;
    out_stream << "Torque: " << rb._torque.x << ", " << rb._torque.y << ", " << rb._torque.z << std::endl;
    return out_stream;
}

RigidBody::~RigidBody() {}

void RigidBody::_calculateRotationMatrix() {
    _rotation_matrix = glm::toMat3(_orientation);
}

glm::mat3 RigidBody::_localToWorldBasisChange(const glm::mat3& local_matrix) const {
    return _rotation_matrix * local_matrix * glm::transpose(_rotation_matrix);
}

glm::mat4 RigidBody::_getLocalToWorldModelMatrix() const {
    glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), _position);
    glm::mat4 rotation_matrix = glm::toMat4(_orientation);
    // glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), _scale);
    return translation_matrix * rotation_matrix;
}

glm::mat4 RigidBody::_getWorldToLocalModelMatrix() const {
    glm::mat4 translation_matrix = glm::translate(glm::mat4(1.0f), -_position);
    glm::mat4 rotation_matrix = glm::toMat4(glm::inverse(_orientation));
    // glm::mat4 scale_matrix = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f / _scale.x, 1.0f / _scale.y, 1.0f / _scale.z));
    return rotation_matrix * translation_matrix;
}

glm::vec3 RigidBody::_getTangentialVelocityOfPoint(const glm::vec3 &point_world_position) const {
    if (!containsPositionInBody(point_world_position)) return glm::vec3(0.0f);
    return glm::cross(getAngularVelocity(), point_world_position);
}
