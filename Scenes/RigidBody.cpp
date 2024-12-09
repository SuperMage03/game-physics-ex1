#include "RigidBody.hpp"

RigidBody::RigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass, const float& elasticity, const float& friction, const glm::vec3& center_of_mass): 
    _transform{position, orientation, scale}, _mass{mass}, _elasticity{elasticity}, _friction{friction}, _center_of_mass{center_of_mass} {}

Transform &RigidBody::getTransform() {
    return _transform;
}

float RigidBody::getElasticity() const {
    return _elasticity;
}

float RigidBody::getFriction() const {
    return _friction;
}

float RigidBody::getMass() const {
    return _mass;
}

glm::vec3 RigidBody::getCenterOfMassWorld() const {
    return _transform.getUnscaledLocalToWorldPosition(_center_of_mass);
}

glm::vec3 RigidBody::getWorldToBodyPosition(const glm::vec3 &world_position) const {
    return world_position - getCenterOfMassWorld();
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

glm::vec3 RigidBody::getAngularVelocity() const {
    return localToWorldBasisChange(glm::inverse(_inertia_tensor)) * _angular_momentum;
}

glm::quat RigidBody::getAngularVelocityQuat() const {
    return glm::quat(0.0f, getAngularVelocity());
}

void RigidBody::setAngularVelocity(const glm::vec3 &angular_velocity) {
    _angular_momentum = localToWorldBasisChange(_inertia_tensor) * angular_velocity;
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

glm::mat3 RigidBody::getInverseInertiaTensorLocal() const {
    return glm::inverse(_inertia_tensor);
}

glm::mat3 RigidBody::getInverseInertiaTensorWorld() const {
    return localToWorldBasisChange(glm::inverse(_inertia_tensor));
}

void RigidBody::addForce(const glm::vec3 &force) {
    _force += force;
}

void RigidBody::addTorque(const glm::vec3 &torque) {
    _torque += torque;
}

// void RigidBody::calculateDerviedData() {}

glm::vec3 RigidBody::getVelocityOfPoint(const glm::vec3 &point_world_position) const {
    return getLinearVelocity() + getTangentialVelocityOfPoint(point_world_position);
}

std::ostream &operator<<(std::ostream &out_stream, const RigidBody &rb) {
    out_stream << rb._transform << std::endl;
    out_stream << "<--------- RigidBody Specific Output --------->" << std::endl;
    out_stream << "Linear Velocity: " << rb._linear_velocity.x << ", " << rb._linear_velocity.y << ", " << rb._linear_velocity.z << std::endl;
    out_stream << "Force: " << rb._force.x << ", " << rb._force.y << ", " << rb._force.z << std::endl;
    out_stream << "Angular Velocity: " << rb.getAngularVelocity().x << ", " << rb.getAngularVelocity().y << ", " << rb.getAngularVelocity().z << std::endl;
    out_stream << "Torque: " << rb._torque.x << ", " << rb._torque.y << ", " << rb._torque.z << std::endl;
    return out_stream;
}

RigidBody::~RigidBody() {}

glm::mat3 RigidBody::localToWorldBasisChange(const glm::mat3& local_matrix) const {
    glm::mat3 rotation_matrix = glm::toMat3(_transform.getOrientation());
    return rotation_matrix * local_matrix * glm::transpose(rotation_matrix);
}

glm::vec3 RigidBody::getTangentialVelocityOfPoint(const glm::vec3 &point_world_position) const {
    if (!containsPositionInBody(point_world_position)) return glm::vec3(0.0f);
    return glm::cross(getAngularVelocity(), getWorldToBodyPosition(point_world_position));
}
