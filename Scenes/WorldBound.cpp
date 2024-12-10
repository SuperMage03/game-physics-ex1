#include "WorldBound.hpp"

WorldBound::WorldBound(const Axis& axis, const Limit& limit, const float& bounding_value) : 
    _bounding_value{bounding_value}, 
    _normal{(limit == Limit::MIN ? 1.0f : -1.0f) * glm::vec3(axis == Axis::X_AXIS, axis == Axis::Y_AXIS, axis == Axis::Z_AXIS)} {}

float WorldBound::getBoundingValue() const {
    return _bounding_value;
}

glm::vec3 WorldBound::getNormal() const {
    return _normal;
}

bool WorldBound::isWithinBound(const glm::vec3& point) const {
    return glm::dot(point, _normal) <= _bounding_value;
}

float WorldBound::distanceOutOfBound(const glm::vec3 &point) const {
    return _bounding_value - glm::dot(point, _normal);
}
