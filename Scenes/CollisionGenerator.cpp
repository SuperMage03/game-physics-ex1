#include "CollisionGenerator.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b)) 
#define MIN(a, b) (((a) > (b)) ? (b) : (a))
#define CLAMP(x, min, max) (MIN(MAX(x, min), max))

CollisionGenerator::~CollisionGenerator() = default;

void BoundingBoxCollisionGenerator::applyCollisionAlongAxis(float &axis_position, const float &axis_pos_min, const float &axis_pos_max, float &axis_velocity) {
    if ((axis_position <= axis_pos_min) || (axis_position >= axis_pos_max)) {
        axis_position = CLAMP(axis_position, axis_pos_min, axis_pos_max);
        axis_velocity *= -1.0f;
    }
}

BoundingBoxCollisionGenerator::BoundingBoxCollisionGenerator(const glm::vec3& box_position, const glm::vec3& box_scale): 
    box_position_{box_position}, box_scale_{box_scale} {}

void BoundingBoxCollisionGenerator::applyCollision(Point &point, float duration) {
    glm::vec3 result_position = point.getPosition();
    glm::vec3 result_velocity = point.getVelocity();

    glm::vec3 min_coordinates = box_position_ - box_scale_ / 2.0f;
    glm::vec3 max_coordinates = box_position_ + box_scale_ / 2.0f;

    applyCollisionAlongAxis(result_position.x, min_coordinates.x, max_coordinates.x, result_velocity.x);
    applyCollisionAlongAxis(result_position.y, min_coordinates.y, max_coordinates.y, result_velocity.y);
    applyCollisionAlongAxis(result_position.z, min_coordinates.z, max_coordinates.z, result_velocity.z);
    point.setPosition(result_position);
    point.setVelocity(result_velocity);
}

CollisionRegistry::CollisionRegistry() = default;

std::unique_ptr<CollisionRegistry> CollisionRegistry::singleton_ = nullptr;

CollisionRegistry::CollisionRegistration::CollisionRegistration(Point *const &point, CollisionGenerator *const &collision_generator): 
    point{point}, collision_generator{collision_generator} {}

CollisionRegistry* CollisionRegistry::getInstance() {
    if (!singleton_) {
        singleton_ = std::unique_ptr<CollisionRegistry>(new CollisionRegistry());
    }
    return singleton_.get();
}

void CollisionRegistry::add(Point &point, CollisionGenerator &collision_generator) {
    registry_.emplace_back(&point, &collision_generator);
}

void CollisionRegistry::remove(Point &point, CollisionGenerator &collision_generator) {
    for (auto i = registry_.begin(); i != registry_.end(); i++) {
        if (i->point != &point) continue;
        if (i->collision_generator != &collision_generator) continue;
        
        registry_.erase(i);
        break;
    }
}

void CollisionRegistry::clear() {
    registry_.clear();
}

void CollisionRegistry::applyCollisions(float duration) {
    for (auto& reg : registry_) {
        reg.collision_generator->applyCollision(*(reg.point), duration);
    }
}
