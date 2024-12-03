#include "ForceRegistry.h"
#include <memory>

ForceRegistry::ForceRegistration::ForceRegistration(RigidBody*const& rb, ForceGenerator*const& force_generator):
    rb{rb}, force_generator{force_generator} {}

ForceRegistry::ForceRegistry() = default;

std::unique_ptr<ForceRegistry> ForceRegistry::singleton_ = nullptr;

ForceRegistry* ForceRegistry::getInstance() {
    if (!singleton_) {
        singleton_ = std::unique_ptr<ForceRegistry>(new ForceRegistry());
    }
    return singleton_.get();
}

void ForceRegistry::add(RigidBody& rb, ForceGenerator& force_generator) {
    registry_.emplace_back(&rb, &force_generator);
}

void ForceRegistry::remove(RigidBody& rb, ForceGenerator& force_generator) {
    for (auto i = registry_.begin(); i != registry_.end(); i++) {
        if (i->rb != &rb) continue;
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
        reg.force_generator->updateForce(*(reg.rb), duration);
    }
}
