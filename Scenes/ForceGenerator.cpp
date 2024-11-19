#include "ForceGenerator.h"
#include <glm/glm.hpp>

// --------- ForceGenerator class method implementations ------------
ForceGenerator::~ForceGenerator() = default;

// --------- GravityForceGenerator class method implementations ------------
GravityForceGenerator::GravityForceGenerator(const glm::vec3 &gravity): gravity_{gravity} {}

void GravityForceGenerator::updateForce(Point& point, float duration) {
    point.addForce(point.getMass() * gravity_);
}

// --------- SpringForceGenerator class method implementations ------------
SpringForceGenerator::SpringForceGenerator(Point& other, float stiffness, float initial_length): 
    other_{other}, stiffness_{stiffness}, initial_length_{initial_length} {}

void SpringForceGenerator::updateForce(Point& point, float duration) {
    // Calculate the current length of the spring
    float current_length = glm::distance(other_.getPosition(), point.getPosition());
    // Calculate the spring force (Hooke's law)
    float spring_force_magnitude = -stiffness_ * (current_length - initial_length_);
    // Normalize the direction of the spring
    glm::vec3 direction = glm::normalize(point.getPosition() - other_.getPosition());
    // Apply the force to the point
    point.addForce(spring_force_magnitude * direction);
}

void DampingForceGenerator::updateForce(Point &point, float duration) {
    point.addForce(-point.getDamping() * point.getVelocity());
}

// --------- ForceRegistry::ForceRegistration struct method implementations ------------
ForceRegistry::ForceRegistration::ForceRegistration(Point*const& point, ForceGenerator*const& force_generator):
    point{point}, force_generator{force_generator} {}

// --------- ForceRegistry class method implementations ------------
ForceRegistry::ForceRegistry() = default;

std::unique_ptr<ForceRegistry> ForceRegistry::singleton_ = nullptr;

ForceRegistry* ForceRegistry::getInstance() {
    if (!singleton_) {
        singleton_ = std::unique_ptr<ForceRegistry>(new ForceRegistry());
    }
    return singleton_.get();
}

void ForceRegistry::add(Point& point, ForceGenerator& force_generator) {
    registry_.emplace_back(&point, &force_generator);
}

void ForceRegistry::remove(Point& point, ForceGenerator& force_generator) {
    for (auto i = registry_.begin(); i != registry_.end(); i++) {
        if (i->point != &point) continue;
        if (i->force_generator != &force_generator) continue;
        
        registry_.erase(i);
        break;
    }
}

void ForceRegistry::clear() {
    registry_.clear();
}

void ForceRegistry::updateForces(float duration) {
    for (auto& reg : registry_) {
        reg.force_generator->updateForce(*(reg.point), duration);
    }
}
