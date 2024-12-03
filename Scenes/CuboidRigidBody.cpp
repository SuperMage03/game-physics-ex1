#include "CuboidRigidBody.h"
#include <cmath>

CuboidRigidBody::CuboidRigidBody(const float& mass, const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation):
    RigidBody{mass, position, scale, orientation} {}

void CuboidRigidBody::initializeData() {
    _calculateRotationMatrix();
    _calculateInertiaTensorCuboid();
}

std::unique_ptr<RigidBody> CuboidRigidBody::clone() {
    return std::make_unique<CuboidRigidBody>(*this);
}

void CuboidRigidBody::_calculateInertiaTensorCuboid() {
    float I_xx = (1.0f / 12.0f) * _mass * (powf(_scale.x, 2.0f) + powf(_scale.z, 2.0f));
    float I_yy = (1.0f / 12.0f) * _mass * (powf(_scale.y, 2.0f) + powf(_scale.z, 2.0f));
    float I_zz = (1.0f / 12.0f) * _mass * (powf(_scale.x, 2.0f) + powf(_scale.y, 2.0f));
    _inertia_tensor = glm::mat3(I_xx, 0.0f, 0.0f,
                                0.0f, I_yy, 0.0f,
                                0.0f, 0.0f, I_zz);
}
