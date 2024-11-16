#pragma once
#include "Renderer.h"
#include <iostream>
#include <iomanip>

class MassPoint {
private:
    uint32_t _id;
    glm::vec3 _position;
    glm::vec3 _velocity;
    glm::vec3 _force;
    float _mass;
    float _damping;

public:
    MassPoint():
    _id(0),
    _position(0.),
    _velocity(0.),
    _force(0.),
    _mass(1.),
    _damping(0.)
    {}

    MassPoint(const MassPoint& other);

    MassPoint(MassPoint&& other);

    MassPoint(uint32_t id);

    MassPoint(uint32_t id, glm::vec3 position, glm::vec3 velocity, float mass, float damping);

    uint32_t getId() const;

    glm::vec3 getPosition() const;

    glm::vec3 getVelocity() const;

    glm::vec3 getForce() const;

    float getMass() const;

    float getDamping() const;

    void setId(uint32_t id);

    void setPosition(glm::vec3 position);

    void setPosition(float x, float y, float z);

    void shiftPosition(glm::vec3 shift);

    void scalePosition(float scale);

    void setVelocity(glm::vec3 velocity);

    void setMass(float mass);

    void setDamping(float damping);

    void applyImpulse(glm::vec3 impulse);

    void clearForce();

    void addForce(glm::vec3 force);

    friend std::ostream& operator<<(std::ostream& os, const MassPoint& massPoint);

    MassPoint& operator=(const MassPoint& other);

    MassPoint& operator=(MassPoint&& other) noexcept;

    void onDraw(Renderer& renderer);
};