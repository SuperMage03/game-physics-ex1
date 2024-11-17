#pragma once
#include <vector>
#include <memory>
#include "Point.h"

class CollisionGenerator {
public:
    virtual ~CollisionGenerator();
    virtual void applyCollision(Point& point, float duration) = 0;
};

class BoundingBoxCollisionGenerator : public CollisionGenerator {
private:
    glm::vec3 box_position_;
    glm::vec3 box_scale_;
    static void applyCollisionAlongAxis(float& axis_position, const float& axis_pos_min, const float& axis_pos_max, float& axis_velocity);
public:
    BoundingBoxCollisionGenerator(const glm::vec3& box_position, const glm::vec3& box_scale);
    void applyCollision(Point& point, float duration) override;
};

// Singleton class for managing collision generators
class CollisionRegistry {
protected:
    CollisionRegistry();
    static std::unique_ptr<CollisionRegistry> singleton_;
    struct CollisionRegistration {
        Point* point;
        CollisionGenerator* collision_generator;
        CollisionRegistration(Point*const& point, CollisionGenerator*const& collision_generator);
    };
    std::vector<CollisionRegistration> registry_;
public:
    static CollisionRegistry* getInstance();

    // Singletons can't cloned nor copied
    CollisionRegistry(const CollisionRegistry&) = delete;
    CollisionRegistry& operator=(const CollisionRegistry&) = delete;
    
    // Methods
    void add(Point& point, CollisionGenerator& collision_generator);
    void remove(Point& point, CollisionGenerator& collision_generator);
    void clear();
    void applyCollisions(float duration=0.0f);
};
