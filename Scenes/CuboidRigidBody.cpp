#include "CuboidRigidBody.h"
#include <cmath>

CuboidRigidBody::CuboidRigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass):
    RigidBody{position, scale, orientation, mass} {
    _calculateInertiaTensorCuboid();
}

std::unique_ptr<RigidBody> CuboidRigidBody::clone() {
    return std::make_unique<CuboidRigidBody>(*this);
}

bool CuboidRigidBody::containsPositionInBody(const glm::vec3 &world_position) const {
    glm::vec3 local_position = worldToScaledLocalPosition(world_position);
    if (fabs(local_position.x) > 0.5f) return false;
    if (fabs(local_position.y) > 0.5f) return false;
    if (fabs(local_position.z) > 0.5f) return false;
    return true;
}

void CuboidRigidBody::_calculateInertiaTensorCuboid() {
    float I_xx = (1.0f / 12.0f) * _mass * (powf(_scale.y, 2.0f) + powf(_scale.z, 2.0f));
    float I_yy = (1.0f / 12.0f) * _mass * (powf(_scale.x, 2.0f) + powf(_scale.z, 2.0f));
    float I_zz = (1.0f / 12.0f) * _mass * (powf(_scale.x, 2.0f) + powf(_scale.y, 2.0f));
    _inertia_tensor = glm::mat3(I_xx, 0.0f, 0.0f,
                                0.0f, I_yy, 0.0f,
                                0.0f, 0.0f, I_zz);
}
