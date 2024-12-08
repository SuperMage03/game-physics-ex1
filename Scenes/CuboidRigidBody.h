#pragma once
#include "RigidBody.h"

class CuboidRigidBody : public RigidBody {
public:
    CuboidRigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass, const float& elasticity = 1.0f, const float& friction = 0.0f);
    std::unique_ptr<RigidBody> clone() override;
    bool containsPositionInBody(const glm::vec3& world_position) const override;
private:
    void calculateInertiaTensorCuboid();
};
