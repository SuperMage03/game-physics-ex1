#include "CollisionSolver.h"
#include <iostream>

void ImpulseSolver::resolveContacts(std::vector<CollisionContact> &contacts, float duration) {
    float elasticity = 0.0f;
    for (const auto& contact : contacts) {
        RigidBody& a = *(contact.a);
        RigidBody& b = *(contact.b);
        const CollisionInfo& collision_info = contact.collision_info;

        glm::vec3 relative_velocity = a.getVelocityOfPoint(collision_info.collisionPointWorld) - b.getVelocityOfPoint(collision_info.collisionPointWorld);
        
        // Bodies are separating
        if (glm::dot(relative_velocity, collision_info.normalWorld) > 0.0f) continue;

        float inverse_mass_a = 1.0f / a.getMass();
        float inverse_mass_b = 1.0f / b.getMass();
        
        glm::vec3 contact_position_body_a = a.getWorldToBodyPosition(collision_info.collisionPointWorld);
        glm::vec3 contact_position_body_b = b.getWorldToBodyPosition(collision_info.collisionPointWorld);

        std::cout << contact_position_body_a.x << ", " << contact_position_body_a.y << ", " << contact_position_body_a.z << std::endl;
        
        float j = glm::dot(-(1.0f + elasticity) * relative_velocity, collision_info.normalWorld) / 
                (inverse_mass_a + inverse_mass_b + 
                glm::dot(glm::cross(a.getInverseInertiaTensorWorld() * glm::cross(contact_position_body_a, collision_info.normalWorld), contact_position_body_a), collision_info.normalWorld) + 
                glm::dot(glm::cross(b.getInverseInertiaTensorWorld() * glm::cross(contact_position_body_b, collision_info.normalWorld), contact_position_body_b), collision_info.normalWorld));

        // std::cout << j << std::endl;
        std::cout << a.getLinearVelocity().x << std::endl;
        std::cout << b.getLinearVelocity().x << std::endl;
        a.setLinearVelocity(a.getLinearVelocity() + (j * inverse_mass_a) * collision_info.normalWorld);
        b.setLinearVelocity(b.getLinearVelocity() - (j * inverse_mass_b) * collision_info.normalWorld);
        std::cout << a.getLinearVelocity().x << std::endl;
        std::cout << b.getLinearVelocity().x << std::endl;
        a.setAngularMomentum(a.getAngularMomentum() + glm::cross(contact_position_body_a, j * collision_info.normalWorld));
        b.setAngularMomentum(b.getAngularMomentum() - glm::cross(contact_position_body_b, j * collision_info.normalWorld));
    }
}
