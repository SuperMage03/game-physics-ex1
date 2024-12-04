#pragma once
#include "RigidBody.h"

class CuboidRigidBody : public RigidBody {
public:
    CuboidRigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass);
    std::unique_ptr<RigidBody> clone() override;
private:
    void _calculateInertiaTensorCuboid();
};
