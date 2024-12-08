#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class Transform {
public:
    glm::vec3 getPosition() const;
    void setPosition(const glm::vec3& position);

    glm::quat getOrientation() const;
    void setOrientation(const glm::quat& orientation);

    glm::vec3 getScale() const;
    void setScale(const glm::vec3& scale);

    glm::mat4 getLocalToWorldMatrix() const;
    glm::mat4 getWorldToLocalMatrix() const;

    glm::vec3 getWorldPosition(const glm::vec3& local_position);
    glm::vec3 getLocalPosition(const glm::vec3& world_position);
private:
    glm::vec3 _position;
    glm::quat _orientation;
    glm::vec3 _scale;
};
