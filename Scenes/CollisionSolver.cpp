#include "CollisionSolver.hpp"
#include <iostream>

void ImpulseSolver::resolveContacts(std::vector<CollisionContact> &contacts, float duration) {
    for (const auto& contact : contacts) {
        RigidBody& a = *(contact.a);
        RigidBody& b = *(contact.b);
        const CollisionInfo& collision_info = contact.collision_info;

        glm::vec3 relative_velocity = a.getVelocityOfPoint(collision_info.collisionPointWorld) - b.getVelocityOfPoint(collision_info.collisionPointWorld);
        
        // Bodies are separating
        if (glm::dot(relative_velocity, collision_info.normalWorld) > 0.0f) continue;

        glm::vec3 tangential_direction = glm::cross(glm::cross(collision_info.normalWorld, relative_velocity), collision_info.normalWorld);
        float tangential_direction_magnitude = glm::length(tangential_direction);
        if (tangential_direction_magnitude > 0.0f) {
            tangential_direction /= tangential_direction_magnitude;
        }

        float inverse_mass_a = 1.0f / a.getMass();
        float inverse_mass_b = 1.0f / b.getMass();
        
        glm::vec3 contact_position_body_a = a.getWorldToBodyPosition(collision_info.collisionPointWorld);
        glm::vec3 contact_position_body_b = b.getWorldToBodyPosition(collision_info.collisionPointWorld);

        std::cout << tangential_direction.x << ", " << tangential_direction.y << ", " << tangential_direction.z << std::endl;
        
        float j = glm::dot(-(1.0f + 0.5f * (a.getElasticity() + b.getElasticity())) * relative_velocity, collision_info.normalWorld) / 
                (inverse_mass_a + inverse_mass_b + 
                glm::dot(glm::cross(a.getInverseInertiaTensorWorld() * glm::cross(contact_position_body_a, collision_info.normalWorld), contact_position_body_a), collision_info.normalWorld) + 
                glm::dot(glm::cross(b.getInverseInertiaTensorWorld() * glm::cross(contact_position_body_b, collision_info.normalWorld), contact_position_body_b), collision_info.normalWorld));

        // std::cout << j << std::endl;
        std::cout << a.getLinearVelocity().x << std::endl;
        std::cout << b.getLinearVelocity().x << std::endl;
        a.setLinearVelocity(a.getLinearVelocity() + (j * inverse_mass_a) * (collision_info.normalWorld - a.getFriction() * tangential_direction));
        b.setLinearVelocity(b.getLinearVelocity() - (j * inverse_mass_b) * (collision_info.normalWorld - b.getFriction() * tangential_direction));
        std::cout << a.getLinearVelocity().x << std::endl;
        std::cout << b.getLinearVelocity().x << std::endl;
        a.setAngularMomentum(a.getAngularMomentum() + glm::cross(contact_position_body_a, j * collision_info.normalWorld));
        b.setAngularMomentum(b.getAngularMomentum() - glm::cross(contact_position_body_b, j * collision_info.normalWorld));
    }
}
