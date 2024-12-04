#pragma once
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody {
public:
    RigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass = 1.0f);
    
    // Getters and Setters
    glm::vec3 getPosition() const;
    void setPosition(const glm::vec3& position);

    glm::vec3 getScale() const;
    void setScale(const glm::vec3& scale);

    glm::vec3 getLinearVelocity() const;
    void setLinearVelocity(const glm::vec3& linear_velocity);

    glm::vec3 getLinearAcceleration() const;
    void setLinearAcceleration(const glm::vec3& linear_acceleration);
    
    glm::vec3 getForce() const;
    void setForce(const glm::vec3& force);

    glm::quat getOrientation() const;
    void setOrientation(const glm::quat& orientation);

    glm::vec3 getAngularVelocity() const;
    glm::quat getAngularVelocityQuat() const;
    void setAngularVelocity(const glm::vec3& angular_velocity);

    glm::vec3 getAngularMomentum() const;
    void setAngularMomentum(const glm::vec3& angular_momentum);

    glm::vec3 getTorque() const;
    void setTorque(const glm::vec3& force);

    void addForce(const glm::vec3& force);
    void addTorque(const glm::vec3& torque);

    // Calculates data that changes depending on position, scale, or orientation
    void calculateDerviedData();
    
    // Virtual Methods
    virtual ~RigidBody();
    virtual std::unique_ptr<RigidBody> clone() = 0;
protected:
    // Private Member Values
    float _mass{1.0f};
    glm::vec3 _position{0.0f};
    glm::vec3 _scale{1.0f};
    glm::quat _orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 _force{0.0f};
    glm::vec3 _linear_velocity{0.0f};
    glm::vec3 _torque{0.0f};
    glm::vec3 _angular_momentum{0.0f};
    // Matrix that converts local to world
    glm::mat3 _rotation_matrix{0.0f};
    // Inertia Tensor in body world
    glm::mat3 _inertia_tensor{0.0f};
    // Private Methods
    void _calculateRotationMatrix();
    glm::mat3 _localToWorldBasisChange(const glm::mat3& local_matrix) const;
};
