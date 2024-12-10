#pragma once
#include <glm/glm.hpp>

class WorldBound {
public:
    enum class Axis {
        X_AXIS,
        Y_AXIS,
        Z_AXIS,
    };
    enum class Limit {
        MIN,
        MAX,
    };
    WorldBound(const Axis& axis, const Limit& limit, const float& bounding_value);
    float getBoundingValue() const;
    glm::vec3 getNormal() const;
    bool isWithinBound(const glm::vec3& point) const;
    float distanceOutOfBound(const glm::vec3& point) const;
private:
    const float _bounding_value{0.0f};
    const glm::vec3 _normal{0.0f};
};
