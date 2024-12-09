#include "ForceGenerator.hpp"

ForceGenerator::~ForceGenerator() {}

GenericTorqueGenerator::GenericTorqueGenerator(const glm::vec3& torque): _torque{torque} {}

void GenericTorqueGenerator::updateForce(RigidBody &rb, float duration) {
    rb.addTorque(_torque);
}

ExternalForceGenerator::ExternalForceGenerator(const glm::vec3& force, const glm::vec3& world_position): _force{force}, _world_position{world_position} {}

void ExternalForceGenerator::updateForce(RigidBody &rb, float duration) {
    if (!rb.containsPositionInBody(_world_position)) return;

    rb.addForce(_force);
    glm::vec3 resulting_torque = glm::cross(rb.getWorldToBodyPosition(_world_position), _force);
    rb.addTorque(resulting_torque);
}

GravityForceGenerator::GravityForceGenerator(const glm::vec3 &gravity_acceleration): _gravity_acceleration{gravity_acceleration} {}

void GravityForceGenerator::updateForce(RigidBody& rb, float duration) {
    rb.addForce(rb.getMass() * _gravity_acceleration);
}
