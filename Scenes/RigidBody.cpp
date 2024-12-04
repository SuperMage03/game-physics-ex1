#include "RigidBody.h"
#include <glm/gtx/quaternion.hpp>

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
    _orientation = orientation;
}

glm::vec3 RigidBody::getAngularVelocity() const {
    return _rotation_matrix * glm::inverse(_inertia_tensor) * glm::transpose(_rotation_matrix) * _angular_momentum;
}

glm::quat RigidBody::getAngularVelocityQuat() const {
    return glm::quat(0.0f, getAngularVelocity());
}

void RigidBody::setAngularVelocity(const glm::vec3 &angular_velocity) {
    _angular_momentum = _rotation_matrix * _inertia_tensor * glm::transpose(_rotation_matrix) * angular_velocity;
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

RigidBody::~RigidBody() {}

void RigidBody::_calculateRotationMatrix() {
    _rotation_matrix = glm::toMat3(glm::normalize(_orientation));
}

