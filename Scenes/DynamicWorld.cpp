#include "DynamicWorld.h"
#include "ForceRegistry.h"

DynamicWorld::DynamicWorld(): _integration_mode{IntegrationMode::EULER} {};

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

void DynamicWorld::initRigidBodyData() {
    for (auto &rb : _registry) {
        rb->initializeData();
    }
}

void DynamicWorld::add(RigidBody &rb) {
    _registry.emplace_back(&rb);
}

void DynamicWorld::remove(RigidBody& rb) {
    for (auto i = _registry.begin(); i != _registry.end(); i++) {
        if (*i != &rb) continue;
        _registry.erase(i);
        break;
    }
}

void DynamicWorld::clear() {
    _registry.clear();
}

void DynamicWorld::clearAccumulators() {
    for (auto& rb : _registry) {
        rb->setForce(glm::vec3(0.0f));
        rb->setTorque(glm::vec3(0.0f));
    }
}

void DynamicWorld::simulateStepEuler(const float& step) {
    // Integrate Position
    for (auto& rb : _registry) {
        rb->setPosition(rb->getPosition() + step * rb->getLinearVelocity());
    }
    // Integrate Velocity
    for (auto& rb : _registry) {
        rb->setLinearVelocity(rb->getLinearVelocity() + step * rb->getLinearAcceleration());
    }

    // Integrate Orientation
    for (auto& rb : _registry) {
        rb->setOrientation(rb->getOrientation() + 0.5f * rb->getOrientation() * rb->getAngularVelocityQuat() * step);
    }
    // Integrate Angular Velocity
    for (auto& rb : _registry) {
        rb->setAngularMomentum(rb->getAngularMomentum() + step * rb->getTorque());
    }
}

void DynamicWorld::simulateStepMidpoint(const float& step) {
    std::unique_ptr<std::unique_ptr<RigidBody>[]> result = std::make_unique<std::unique_ptr<RigidBody>[]>(_registry.size());

    // Store old point data
    for (int i = 0; i < _registry.size(); i++) {
        result[i] = std::move(_registry[i]->clone());
    }

    // Integrate Position
    for (int i = 0; i < _registry.size(); i++) {
        RigidBody& simulated_body = *(_registry[i]);
        RigidBody& result_body = *(result[i]);
        
        simulated_body.setPosition(simulated_body.getPosition() + (step / 2) * simulated_body.getLinearVelocity());
        simulated_body.setLinearVelocity(simulated_body.getLinearVelocity() + (step / 2) * simulated_body.getLinearAcceleration());

        simulated_body.setOrientation(simulated_body.getOrientation() + 0.5f * simulated_body.getOrientation() * simulated_body.getAngularVelocityQuat() * (step / 2));
        simulated_body.setAngularMomentum(simulated_body.getAngularMomentum() + (step / 2) * simulated_body.getTorque());

        // Calculated result position and orientation from the velocity at step / 2
        result_body.setPosition(result_body.getPosition() + step * result_body.getLinearVelocity());
        result_body.setOrientation(result_body.getOrientation() + 0.5f * result_body.getOrientation() *  result_body.getAngularVelocityQuat() * step);
    }

    // Check Collision
    // CollisionRegistry::getInstance()->applyCollisions();
    
    // Let the forces and torques to be recalculated
    clearAccumulators();
    ForceRegistry::getInstance()->updateForces();
    // Calculate derived data
    for (auto& rb : _registry) {
        rb->calculateDerviedData();
    }

    // Integrate Velocity
    for (int i = 0; i < _registry.size(); i++) {
        RigidBody& simulated_body = *(_registry[i]);
        RigidBody& result_body = *(result[i]);

        // Calculated result linear and angular velocity from the force at step / 2
        result_body.setLinearVelocity(result_body.getLinearVelocity() + step * simulated_body.getLinearAcceleration());
        result_body.setAngularMomentum(result_body.getAngularMomentum() + step * simulated_body.getTorque());
    }

    // Finally setting the result to simulated point
    for (int i = 0; i < _registry.size(); i++) {
        RigidBody& simulated_body = *(_registry[i]);
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
    // Calculate derived data
    for (auto& rb : _registry) {
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
