#include "Point.h"
#include "ForceGenerator.h"
#include "CollisionGenerator.h"

// --------- Point class method implementations ------------
Point::Point(float mass): 
    position_{glm::vec3(0)}, velocity_{glm::vec3(0)}, force_{glm::vec3(0)}, mass_{mass}, damping_{0.0f} {}

Point::Point(glm::vec3 position, glm::vec3 velocity, glm::vec3 force, float mass, float damping): 
    position_{position}, velocity_{velocity}, force_{force}, mass_{mass}, damping_{damping} {}

float Point::getMass() const {
    return mass_;
}

glm::vec3 Point::getPosition() const {
    return position_;
}

glm::vec3 Point::getVelocity() const {
    return velocity_;
}

glm::vec3 Point::getForce() const {
    return force_;
}

glm::vec3 Point::getAcceleration() const {
    if (mass_ == 0.0f) {
        return glm::vec3(0.0f); // Avoid division by zero if mass is zero
    }
    return force_ / mass_;
}

void Point::setPosition(const glm::vec3& position) {
    position_ = position;
}

void Point::setVelocity(const glm::vec3& velocity) {
    velocity_ = velocity;
}

void Point::setForce(const glm::vec3& force) {
    force_ = force;
}

void Point::setAcceleration(const glm::vec3& acceleration) {
    force_ = mass_ * acceleration;
}

void Point::addForce(const glm::vec3& force) {
    force_ += force;
}

std::ostream& operator<<(std::ostream& out_stream, const Point& point) {
    out_stream << "Position: " << point.position_.x << ", " << point.position_.y << ", " << point.position_.z << std::endl;
    out_stream << "Velocity: " << point.velocity_.x << ", " << point.velocity_.y << ", " << point.velocity_.z << std::endl;
    out_stream << "Force: " << point.force_.x << ", " << point.force_.y << ", " << point.force_.z << std::endl;
    return out_stream;
}

// --------- PointRegistry::PointRegistration struct method implementations ------------
PointRegistry::PointRegistration::PointRegistration(Point*const& point): point{point} {}

// --------- ForceRegistry class method implementations ------------
PointRegistry::PointRegistry(): integration_mode_{IntegrationMode::EULER} {};

std::unique_ptr<PointRegistry> PointRegistry::singleton_ = nullptr;

PointRegistry* PointRegistry::getInstance() {
    if (!singleton_) {
        singleton_ = std::unique_ptr<PointRegistry>(new PointRegistry());
    }
    return singleton_.get();
}

PointRegistry::IntegrationMode PointRegistry::getIntegrationMode() const {
    return integration_mode_;
}

void PointRegistry::setIntegrationMode(const PointRegistry::IntegrationMode& integration_mode) {
    integration_mode_ = integration_mode;
}

void PointRegistry::add(Point &point) {
    registry_.emplace_back(&point);
}

void PointRegistry::remove(Point& point) {
    for (auto i = registry_.begin(); i != registry_.end(); i++) {
        if (i->point != &point) continue;
        
        registry_.erase(i);
        break;
    }
}

void PointRegistry::clear() {
    registry_.clear();
}

void PointRegistry::clearForces() {
    for (auto& reg : registry_) {
        reg.point->setForce(glm::vec3(0.0f));
    }
}


void PointRegistry::simulateStepEuler(const float& step) {
    // Integrate Position
    for (auto& reg : registry_) {
        reg.point->setPosition(reg.point->getPosition() + step * reg.point->getVelocity());
    }
    // Integrate Velocity
    for (auto& reg : registry_) {
        reg.point->setVelocity(reg.point->getVelocity() + step * reg.point->getAcceleration());
    }
}

void PointRegistry::simulateStepMidpoint(const float& step) {
    std::unique_ptr<Point[]> result = std::make_unique<Point[]>(registry_.size());

    // Store old point data
    for (int i = 0; i < registry_.size(); i++) {
        result[i] = *(registry_[i].point);
    }

    // Integrate Position
    for (int i = 0; i < registry_.size(); i++) {
        Point& simulated_point = *(registry_[i].point);
        Point& result_point = result[i];
        
        simulated_point.setPosition(simulated_point.getPosition() + (step / 2) * simulated_point.getVelocity());
        simulated_point.setVelocity(simulated_point.getVelocity() + (step / 2) * simulated_point.getAcceleration());

        // Calculated result position from the velocity at step / 2
        result_point.setPosition(result_point.getPosition() + step * simulated_point.getVelocity());
    }

    // Check Collision
    CollisionRegistry::getInstance()->applyCollisions();
    // Let the forces to be recalculated
    clearForces();
    ForceRegistry::getInstance()->updateForces();

    // Integrate Velocity
    for (int i = 0; i < registry_.size(); i++) {
        Point& simulated_point = *(registry_[i].point);
        Point& result_point = result[i];

        // Calculated result velocity from the force at step / 2
        result_point.setVelocity(result_point.getVelocity() + step * simulated_point.getAcceleration());
    }

    // Finally setting the result to simulated point
    for (int i = 0; i < registry_.size(); i++) {
        Point& simulated_point = *(registry_[i].point);
        Point& result_point = result[i];
        simulated_point = result_point;
    }
}

void PointRegistry::simulateStep(const float& step) {
    // Check Collision
    CollisionRegistry::getInstance()->applyCollisions();
    // Calculate the forces
    clearForces();
    ForceRegistry::getInstance()->updateForces();

    switch (integration_mode_) {
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
