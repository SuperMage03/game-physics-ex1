#pragma once
#include <vector>
#include <memory>
#include "RigidBody.h"

class DynamicWorld {
public:
    enum class IntegrationMode {
        EULER,
        MIDPOINT,
    };

    static DynamicWorld* getInstance();

    // Singletons can't cloned nor copied
    DynamicWorld(const DynamicWorld&) = delete;
    DynamicWorld& operator=(const DynamicWorld&) = delete;

    // Getters
    IntegrationMode getIntegrationMode() const;

    // Setters
    void setIntegrationMode(const IntegrationMode& integration_mode);

    // Methods
    void initRigidBodyData();
    void add(RigidBody& rb);
    void remove(RigidBody& rb);
    void clear();
    void clearAccumulators();
    void simulateStep(const float& step=0.1f);
protected:
    static std::unique_ptr<DynamicWorld> _singleton;
    
    std::vector<RigidBody*> _registry;
    IntegrationMode _integration_mode;

    DynamicWorld();
    void simulateStepEuler(const float& step);
    void simulateStepMidpoint(const float& step);
};
