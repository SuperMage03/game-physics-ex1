#include "DynamicWorld.hpp"
#include <util/CollisionDetection.h>
#include "ForceRegistry.hpp"

DynamicWorld::DynamicWorld(): _integration_mode{IntegrationMode::EULER} {}

std::unique_ptr<DynamicWorld> DynamicWorld::_singleton = nullptr;

DynamicWorld* DynamicWorld::getInstance() {
    if (!_singleton) {
        _singleton = std::unique_ptr<DynamicWorld>(new DynamicWorld());
    }
    return _singleton.get();
}

DynamicWorld::IntegrationMode DynamicWorld::getIntegrationMode() const {
    return _integration_mode;
}

void DynamicWorld::setIntegrationMode(const DynamicWorld::IntegrationMode& integration_mode) {
    _integration_mode = integration_mode;
}

void DynamicWorld::addDynamicObject(RigidBody &rb) {
    _dynamic_objects.emplace_back(&rb);
}

void DynamicWorld::removeDynamicObject(RigidBody& rb) {
    for (auto i = _dynamic_objects.begin(); i != _dynamic_objects.end(); i++) {
        if (*i != &rb) continue;
        _dynamic_objects.erase(i);
        break;
    }
}

void DynamicWorld::addWorldBound(WorldBound &bound) {
    _world_bounds.emplace_back(&bound);
}

void DynamicWorld::removeWorldBound(WorldBound &bound) {
    for (auto i = _world_bounds.begin(); i != _world_bounds.end(); i++) {
        if (*i != &bound) continue;
        _world_bounds.erase(i);
        break;
    }
}

void DynamicWorld::addCollidableObject(RigidBody &rb) {
    _collidable_objects.emplace_back(&rb);
}

void DynamicWorld::removeCollidableObject(RigidBody& rb) {
    for (auto i = _collidable_objects.begin(); i != _collidable_objects.end(); i++) {
        if (*i != &rb) continue;
        _collidable_objects.erase(i);
        break;
    }
}

void DynamicWorld::addCollisionSolver(CollisionSolver& collision_solver) {
    _collision_solvers.emplace_back(&collision_solver);
}

void DynamicWorld::removeCollisionSolver(CollisionSolver& collision_solver) {
    for (auto i = _collision_solvers.begin(); i != _collision_solvers.end(); i++) {
        if (*i != &collision_solver) continue;
        _collision_solvers.erase(i);
        break;
    }
}

void DynamicWorld::clear() {
    _world_bounds.clear();
    _collidable_objects.clear();
    _dynamic_objects.clear();
    _collision_contacts.clear();
    _collision_solvers.clear();
}

void DynamicWorld::clearAccumulators() {
    for (auto& rb : _dynamic_objects) {
        rb->setForce(glm::vec3(0.0f));
        rb->setTorque(glm::vec3(0.0f));
    }
}

void DynamicWorld::simulateStepEuler(const float& step) {
    // Integrate Position
    for (auto& rb : _dynamic_objects) {
        Transform& rb_transform = rb->getTransform();
        rb_transform.setPosition(rb_transform.getPosition() + step * rb->getLinearVelocity());
    }
    // Integrate Velocity
    for (auto& rb : _dynamic_objects) {
        rb->setLinearVelocity(rb->getLinearVelocity() + step * rb->getLinearAcceleration());
    }

    // Integrate Orientation
    for (auto& rb : _dynamic_objects) {
        Transform& rb_transform = rb->getTransform();
        rb_transform.setOrientation(rb_transform.getOrientation() + 0.5f * rb->getAngularVelocityQuat() * rb_transform.getOrientation() * step);
    }
    // Integrate Angular Velocity
    for (auto& rb : _dynamic_objects) {
        rb->setAngularMomentum(rb->getAngularMomentum() + step * rb->getTorque());
    }
}

void DynamicWorld::simulateStepMidpoint(const float& step) {
    std::unique_ptr<std::unique_ptr<RigidBody>[]> result = std::make_unique<std::unique_ptr<RigidBody>[]>(_dynamic_objects.size());

    // Store old point data
    for (int i = 0; i < _dynamic_objects.size(); i++) {
        result[i] = _dynamic_objects[i]->clone();
    }

    // Integrate Position
    for (int i = 0; i < _dynamic_objects.size(); i++) {
        RigidBody& simulated_body = *(_dynamic_objects[i]);
        RigidBody& result_body = *(result[i]);

        Transform& simulated_body_transform = simulated_body.getTransform();
        Transform& result_body_transform = result_body.getTransform();

        simulated_body_transform.setPosition(simulated_body_transform.getPosition() + (step / 2) * simulated_body.getLinearVelocity());
        simulated_body.setLinearVelocity(simulated_body.getLinearVelocity() + (step / 2) * simulated_body.getLinearAcceleration());

        simulated_body_transform.setOrientation(simulated_body_transform.getOrientation() + 0.5f * simulated_body.getAngularVelocityQuat() * simulated_body_transform.getOrientation() * (step / 2.0f));
        simulated_body.setAngularMomentum(simulated_body.getAngularMomentum() + (step / 2) * simulated_body.getTorque());

        // Calculated result position and orientation from the velocity at step / 2
        result_body_transform.setPosition(result_body_transform.getPosition() + step * result_body.getLinearVelocity());
        result_body_transform.setOrientation(result_body_transform.getOrientation() + 0.5f * result_body.getAngularVelocityQuat() * result_body_transform.getOrientation() * step);
    }

    _collision_contacts.clear();
    detectCollisions();
    resolveCollisions();
    applyBoundCollisions();
    
    // Let the forces and torques to be recalculated
    clearAccumulators();
    ForceRegistry::getInstance()->updateForces();

    // Calculate derived data (can be implemented if needed)
    // for (auto& rb : _objects) {
    //     rb->calculateDerviedData();
    // }

    // Integrate Velocity
    for (int i = 0; i < _dynamic_objects.size(); i++) {
        RigidBody& simulated_body = *(_dynamic_objects[i]);
        RigidBody& result_body = *(result[i]);

        // Calculated result linear and angular velocity from the force at step / 2
        result_body.setLinearVelocity(result_body.getLinearVelocity() + step * simulated_body.getLinearAcceleration());
        result_body.setAngularMomentum(result_body.getAngularMomentum() + step * simulated_body.getTorque());
    }

    // Finally setting the result to simulated point
    for (int i = 0; i < _dynamic_objects.size(); i++) {
        RigidBody& simulated_body = *(_dynamic_objects[i]);
        RigidBody& result_body = *(result[i]);
        simulated_body = result_body;
    }
}

void DynamicWorld::simulateStep(const float& step) {
    // Calculate the forces and torques
    clearAccumulators();
    ForceRegistry::getInstance()->updateForces();

    // Calculate derived data (can be implemented if needed)
    // for (auto& rb : _objects) {
    //     rb->calculateDerviedData();
    // }

    switch (_integration_mode) {
    case IntegrationMode::EULER:
        simulateStepEuler(step);
        break;
    case IntegrationMode::MIDPOINT:
        simulateStepMidpoint(step);
        break;
    default:
        break;
    }

    _collision_contacts.clear();
    detectCollisions();
    resolveCollisions();
    applyBoundCollisions();
}

void DynamicWorld::detectCollisions() {
    for (auto i = _collidable_objects.begin(); i < _collidable_objects.end(); i++) {
        for (auto j = i + 1; j < _collidable_objects.end(); j++) {
            CollisionInfo collision_result = collisionTools::checkCollisionSAT((*i)->getTransform().getScaledLocalToWorldMatrix(), (*j)->getTransform().getScaledLocalToWorldMatrix());
            if (collision_result.isColliding) {
                _collision_contacts.emplace_back(*i, *j, collision_result);
            }
        }
    }
}

void DynamicWorld::resolveCollisions() {
    for (auto& collision_solver : _collision_solvers) {
        collision_solver->resolveContacts(_collision_contacts, 0.0f);
    }
}
void DynamicWorld::applyBoundCollisions() {
    for (auto& object : _collidable_objects) {
        for (auto& bound : _world_bounds) {
            std::vector<glm::vec3> vertices_positions = object->getVerticesWorldPositions();
            float max_out_of_bound_distance = -1.0f;
            glm::vec3 max_out_of_bound_vertex_position = glm::vec3(0.0f);
            for (const auto& vertices_position : vertices_positions) {
                float dist_out_of_bound = bound->distanceOutOfBound(vertices_position);
                if (max_out_of_bound_distance < dist_out_of_bound) {
                    max_out_of_bound_distance = dist_out_of_bound;
                    max_out_of_bound_vertex_position = vertices_position;
                }
            }

            if (max_out_of_bound_distance < 0.0f) continue;

            glm::vec3 relative_velocity = object->getVelocityOfPoint(max_out_of_bound_vertex_position);
            glm::vec3 contact_position_body = object->getWorldToBodyPosition(max_out_of_bound_vertex_position);

            // Bodies are separating
            if (glm::dot(relative_velocity, bound->getNormal()) > 0.0f) return;

            float j = glm::dot(-(1.0f + object->getElasticity()) * relative_velocity, bound->getNormal()) / 
                    (object->getInverseMass() + 
                     glm::dot(glm::cross(object->getInverseInertiaTensorWorld() * glm::cross(contact_position_body, bound->getNormal()), contact_position_body), bound->getNormal()));

            glm::vec3 tangential_direction = glm::cross(glm::cross(bound->getNormal(), relative_velocity), bound->getNormal());
            float tangential_direction_magnitude = glm::length(tangential_direction);
            if (tangential_direction_magnitude > 0.0f) {
                tangential_direction /= tangential_direction_magnitude;
            }

            object->setLinearVelocity(object->getLinearVelocity() + (j * object->getInverseMass()) * (bound->getNormal() - object->getFriction() * tangential_direction));
            object->setAngularMomentum(object->getAngularMomentum() + glm::cross(contact_position_body, j * bound->getNormal()));

            glm::vec3 fixing_vector = bound->getNormal() * max_out_of_bound_distance;
            object->getTransform().setPosition(object->getTransform().getPosition() + fixing_vector);
        }
    }
};
