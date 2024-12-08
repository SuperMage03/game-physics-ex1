#include "DynamicWorld.h"
#include <util/CollisionDetection.h>
#include "ForceRegistry.h"

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

void DynamicWorld::addObject(RigidBody &rb) {
    _objects.emplace_back(&rb);
}

void DynamicWorld::removeObject(RigidBody& rb) {
    for (auto i = _objects.begin(); i != _objects.end(); i++) {
        if (*i != &rb) continue;
        _objects.erase(i);
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
    _objects.clear();
    _collision_solvers.clear();
    _collision_contacts.clear();
}

void DynamicWorld::clearAccumulators() {
    for (auto& rb : _objects) {
        rb->setForce(glm::vec3(0.0f));
        rb->setTorque(glm::vec3(0.0f));
    }
}

void DynamicWorld::simulateStepEuler(const float& step) {
    // Integrate Position
    for (auto& rb : _objects) {
        rb->setPosition(rb->getPosition() + step * rb->getLinearVelocity());
    }
    // Integrate Velocity
    for (auto& rb : _objects) {
        rb->setLinearVelocity(rb->getLinearVelocity() + step * rb->getLinearAcceleration());
    }

    // Integrate Orientation
    for (auto& rb : _objects) {
        rb->setOrientation(rb->getOrientation() + 0.5f * rb->getAngularVelocityQuat() * rb->getOrientation() * step);
    }
    // Integrate Angular Velocity
    for (auto& rb : _objects) {
        rb->setAngularMomentum(rb->getAngularMomentum() + step * rb->getTorque());
    }
}

void DynamicWorld::simulateStepMidpoint(const float& step) {
    std::unique_ptr<std::unique_ptr<RigidBody>[]> result = std::make_unique<std::unique_ptr<RigidBody>[]>(_objects.size());

    // Store old point data
    for (int i = 0; i < _objects.size(); i++) {
        result[i] = _objects[i]->clone();
    }

    // Integrate Position
    for (int i = 0; i < _objects.size(); i++) {
        RigidBody& simulated_body = *(_objects[i]);
        RigidBody& result_body = *(result[i]);
        
        simulated_body.setPosition(simulated_body.getPosition() + (step / 2) * simulated_body.getLinearVelocity());
        simulated_body.setLinearVelocity(simulated_body.getLinearVelocity() + (step / 2) * simulated_body.getLinearAcceleration());

        simulated_body.setOrientation(simulated_body.getOrientation() + 0.5f * simulated_body.getAngularVelocityQuat() * simulated_body.getOrientation() * (step / 2));
        simulated_body.setAngularMomentum(simulated_body.getAngularMomentum() + (step / 2) * simulated_body.getTorque());

        // Calculated result position and orientation from the velocity at step / 2
        result_body.setPosition(result_body.getPosition() + step * result_body.getLinearVelocity());
        result_body.setOrientation(result_body.getOrientation() + 0.5f * result_body.getAngularVelocityQuat() * result_body.getOrientation() * step);
    }

    // Check Collision
    // CollisionRegistry::getInstance()->applyCollisions();
    
    // Let the forces and torques to be recalculated
    clearAccumulators();
    ForceRegistry::getInstance()->updateForces();
    
    _collision_contacts.clear();
    detectCollisions();
    resolveCollisions();

    // Calculate derived data
    for (auto& rb : _objects) {
        rb->calculateDerviedData();
    }

    // Integrate Velocity
    for (int i = 0; i < _objects.size(); i++) {
        RigidBody& simulated_body = *(_objects[i]);
        RigidBody& result_body = *(result[i]);

        // Calculated result linear and angular velocity from the force at step / 2
        result_body.setLinearVelocity(result_body.getLinearVelocity() + step * simulated_body.getLinearAcceleration());
        result_body.setAngularMomentum(result_body.getAngularMomentum() + step * simulated_body.getTorque());
    }

    // Finally setting the result to simulated point
    for (int i = 0; i < _objects.size(); i++) {
        RigidBody& simulated_body = *(_objects[i]);
        RigidBody& result_body = *(result[i]);
        simulated_body = result_body;
    }
}

void DynamicWorld::simulateStep(const float& step) {
    // Check Collision
    // CollisionRegistry::getInstance()->applyCollisions();
    // Calculate the forces and torques
    clearAccumulators();
    ForceRegistry::getInstance()->updateForces();

    _collision_contacts.clear();
    detectCollisions();
    resolveCollisions();

    // Calculate derived data
    for (auto& rb : _objects) {
        rb->calculateDerviedData();
    }

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
}

void DynamicWorld::detectCollisions() {
    for (auto i = _objects.begin(); i < _objects.end(); i++) {
        for (auto j = i + 1; j < _objects.end(); j++) {
            CollisionInfo collision_result = collisionTools::checkCollisionSAT((*i)->getScaledLocalToWorldMatrix(), (*j)->getScaledLocalToWorldMatrix());
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
};
