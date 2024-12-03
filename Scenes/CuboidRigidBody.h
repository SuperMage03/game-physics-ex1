#pragma once
#include "RigidBody.h"

class CuboidRigidBody : public RigidBody {
public:
    CuboidRigidBody(const float& mass, const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation);
    void initializeData() override;
    std::unique_ptr<RigidBody> clone() override;
private:
    void _calculateInertiaTensorCuboid();
};
