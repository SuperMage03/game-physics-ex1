#pragma once
#include <vector>
#include <memory>
#include "RigidBody.hpp"
#include "CollisionSolver.hpp"
#include "CollisionContact.hpp"

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
    void addCollidableObject(RigidBody& rb);
    void removeCollidableObject(RigidBody& rb);
    void addCollisionSolver(CollisionSolver& collision_solver);
    void removeCollisionSolver(CollisionSolver& collision_solver);

    void addDynamicObject(RigidBody& rb);
    void removeDynamicObject(RigidBody& rb);

    void clear();
    void simulateStep(const float& step=0.1f);
protected:
    static std::unique_ptr<DynamicWorld> _singleton;
    
    std::vector<RigidBody*> _collidable_objects;
    std::vector<RigidBody*> _dynamic_objects;
    
    std::vector<CollisionSolver*> _collision_solvers;
    std::vector<CollisionContact> _collision_contacts;

    IntegrationMode _integration_mode;

    DynamicWorld();
    void clearAccumulators();
    void simulateStepEuler(const float& step);
    void simulateStepMidpoint(const float& step);
    void detectCollisions();
    void resolveCollisions();
};
