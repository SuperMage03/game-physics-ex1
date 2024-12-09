#pragma once
#include <ostream>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Transform.hpp"

/*
Body Space = World Space - Center of Mass in World Space
*/

class RigidBody {
public:
    RigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const float& mass = 1.0f, const float& elasticity = 1.0f, const float& friction = 0.0f, const bool& is_dynamic = true, const glm::vec3& center_of_mass = glm::vec3(0.0f));
    
    // Getters and Setters
    Transform& getTransform();

    float getMass() const;
    float getInverseMass() const;
    float getElasticity() const;
    float getFriction() const;
    bool isDynamic() const;

    glm::vec3 getCenterOfMassWorld() const;
    glm::vec3 getWorldToBodyPosition(const glm::vec3& world_position) const;

    glm::vec3 getLinearVelocity() const;
    void setLinearVelocity(const glm::vec3& linear_velocity);

    glm::vec3 getLinearAcceleration() const;
    void setLinearAcceleration(const glm::vec3& linear_acceleration);
    
    glm::vec3 getForce() const;
    void setForce(const glm::vec3& force);

    glm::vec3 getAngularVelocity() const;
    glm::quat getAngularVelocityQuat() const;
    void setAngularVelocity(const glm::vec3& angular_velocity);

    glm::vec3 getAngularMomentum() const;
    void setAngularMomentum(const glm::vec3& angular_momentum);

    glm::vec3 getTorque() const;
    void setTorque(const glm::vec3& force);

    glm::mat3 getInertiaTensorLocal() const;
    glm::mat3 getInertiaTensorWorld() const;

    glm::mat3 getInverseInertiaTensorLocal() const;
    glm::mat3 getInverseInertiaTensorWorld() const;

    void addForce(const glm::vec3& force);
    void addTorque(const glm::vec3& torque);

    // Calculates data that changes depending on position, scale, or orientation (can be implemented if needed)
    // void calculateDerviedData();

    glm::vec3 getVelocityOfPoint(const glm::vec3& point_world_position) const;

    // Friend Functions
    friend std::ostream& operator<< (std::ostream& out_stream, const RigidBody& rb);

    // Virtual Methods
    virtual ~RigidBody();
    virtual std::unique_ptr<RigidBody> clone() = 0;
    virtual bool containsPositionInBody(const glm::vec3& world_position) const = 0;
protected:
    // Private Member Values
    Transform _transform;
    
    float _mass{1.0f};
    float _elasticity{1.0f}; // 0 (Inelastic) to 1 (Elastic)
    float _friction{0.0f};
    bool _is_dynamic{true};

    glm::vec3 _center_of_mass{0.0f}; // In unscaled local space
    
    glm::vec3 _force{0.0f}; // Body space
    glm::vec3 _linear_velocity{0.0f}; // Body space
    glm::vec3 _torque{0.0f}; // Body space
    glm::vec3 _angular_momentum{0.0f}; // Body space
    
    // Inertia Tensor in body space
    glm::mat3 _inertia_tensor{0.0f};
    glm::mat3 _inverse_inertia_tensor{0.0f};

    // Private Methods
    glm::mat3 localToWorldBasisChange(const glm::mat3& local_matrix) const;
    glm::vec3 getTangentialVelocityOfPoint(const glm::vec3& point_world_position) const;
};
