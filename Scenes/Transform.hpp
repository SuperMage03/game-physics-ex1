#pragma once
#include <ostream>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class Transform {
public:
    Transform(const glm::vec3& positon, const glm::quat& orientation, const glm::vec3& scale);

    glm::vec3 getPosition() const;
    void setPosition(const glm::vec3& position);

    glm::quat getOrientation() const;
    void setOrientation(const glm::quat& orientation);

    glm::vec3 getScale() const;
    void setScale(const glm::vec3& scale);

    glm::mat4 getUnscaledLocalToWorldMatrix() const;
    glm::mat4 getWorldToUnscaledLocalMatrix() const;
    glm::vec3 getUnscaledLocalToWorldPosition(const glm::vec3& position) const;
    glm::vec3 getWorldToUnscaledLocalPosition(const glm::vec3& position) const;

    glm::mat4 getScaledLocalToWorldMatrix() const;
    glm::mat4 getWorldToScaledLocalMatrix() const;
    glm::vec3 getScaledLocalToWorldPosition(const glm::vec3& position) const;
    glm::vec3 getWorldToScaledLocalPosition(const glm::vec3& position) const;

    // Friend Functions
    friend std::ostream& operator<< (std::ostream& out_stream, const Transform& transform);
private:
    glm::vec3 _position;
    glm::quat _orientation;
    glm::vec3 _scale;
};
