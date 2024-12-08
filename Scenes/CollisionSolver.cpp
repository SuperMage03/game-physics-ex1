#include "CollisionSolver.h"

void ImpulseSolver::resolveContacts(std::vector<CollisionContact> &contacts, float duration) {
    float elasticity = 1.0f;
    for (const auto& contact : contacts) {
        RigidBody& a = *(contact.a);
        RigidBody& b = *(contact.b);
        const CollisionInfo& collision_info = contact.collision_info;

        float inverse_mass_a = 1.0f / a.getMass();
        float inverse_mass_b = 1.0f / b.getMass();
        
        glm::vec3 contact_position_local_a = a.worldToUnscaledLocalPosition(collision_info.collisionPointWorld);
        glm::vec3 contact_position_local_b = b.worldToUnscaledLocalPosition(collision_info.collisionPointWorld);
        
        glm::vec3 relative_velocity = a.getVelocityOfPoint(collision_info.collisionPointWorld) - b.getVelocityOfPoint(collision_info.collisionPointWorld);
        float j = glm::dot(-(1 + elasticity) * relative_velocity, collision_info.normalWorld) / 
                (inverse_mass_a + inverse_mass_b + 
                glm::dot(glm::cross(a.getInverseInertiaTensorWorld() * glm::cross(contact_position_local_a, collision_info.normalWorld), contact_position_local_a), collision_info.normalWorld) + 
                glm::dot(glm::cross(b.getInverseInertiaTensorWorld() * glm::cross(contact_position_local_b, collision_info.normalWorld), contact_position_local_b), collision_info.normalWorld));

        a.setLinearVelocity(a.getLinearVelocity() + (j / inverse_mass_a) * collision_info.normalWorld);
        b.setLinearVelocity(b.getLinearVelocity() - (j / inverse_mass_b) * collision_info.normalWorld);
        a.setAngularMomentum(a.getAngularMomentum() + glm::cross(contact_position_local_a, j * collision_info.normalWorld));
        b.setAngularMomentum(b.getAngularMomentum() - glm::cross(contact_position_local_b, j * collision_info.normalWorld));
    }
}
