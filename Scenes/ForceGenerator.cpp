#include "ForceGenerator.h"

ForceGenerator::~ForceGenerator() {}

GenericTorqueGenerator::GenericTorqueGenerator(glm::vec3 torque): _torque{torque} {}

void GenericTorqueGenerator::updateForce(RigidBody &rb, float duration) {
    rb.addTorque(_torque);
}
