#pragma once
#include <vector>
#include "RigidBody.h"
#include "ForceGenerator.h"

// Singleton class for managing force generators
class ForceRegistry {
protected:
    ForceRegistry();
    static std::unique_ptr<ForceRegistry> singleton_;
    struct ForceRegistration {
        RigidBody* rb;
        ForceGenerator* force_generator;
        ForceRegistration(RigidBody*const& rb, ForceGenerator*const& force_generator);
    };
    std::vector<ForceRegistration> registry_;
public:
    static ForceRegistry* getInstance();

    // Singletons can't cloned nor copied
    ForceRegistry(const ForceRegistry&) = delete;
    ForceRegistry& operator=(const ForceRegistry&) = delete;
    
    // Methods
    void add(RigidBody& rb, ForceGenerator& force_generator);
    void remove(RigidBody& rb, ForceGenerator& force_generator);
    void clear();
    void updateForces(float duration=0.0f);
};
