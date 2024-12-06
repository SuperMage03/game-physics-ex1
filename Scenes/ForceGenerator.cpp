#include "ForceGenerator.h"

ForceGenerator::~ForceGenerator() {}

GenericTorqueGenerator::GenericTorqueGenerator(const glm::vec3& torque): _torque{torque} {}

void GenericTorqueGenerator::updateForce(RigidBody &rb, float duration) {
    rb.addTorque(_torque);
}

ExternalForceGenerator::ExternalForceGenerator(const glm::vec3& force, const glm::vec3& world_position): _force{force}, _world_position{world_position} {}

void ExternalForceGenerator::updateForce(RigidBody &rb, float duration) {
    if (!rb.containsPositionInBody(_world_position)) return;

    rb.addForce(_force);
    glm::vec3 resulting_torque = glm::cross(_world_position - rb.getPosition(), _force);
    rb.addTorque(resulting_torque);
}
