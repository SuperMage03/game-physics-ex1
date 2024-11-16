#include "MassPoint.h"

MassPoint::MassPoint(const MassPoint& other) {
    this->_id = other._id;
    this->_position = other._position;
    this->_velocity = other._velocity;
    this->_force = other._force;
    this->_mass = other._mass;
    this->_damping = other._damping;
}

MassPoint::MassPoint(MassPoint&& other) {
    this->_id = other._id;
    this->_position = std::move(other._position);
    this->_velocity = std::move(other._velocity);
    this->_force = std::move(other._force);
    this->_mass = other._mass;
    this->_damping = other._damping;
}

MassPoint::MassPoint(uint32_t id): MassPoint() {
    this->_id = id;
}

MassPoint::MassPoint(uint32_t id, glm::vec3 position, glm::vec3 velocity, float mass, float damping): MassPoint(id) {
    this->_position = position;
    this->_velocity = velocity;
    this->_mass = mass;
    this->_damping = damping;
}

uint32_t MassPoint::getId() const {
    return this->_id;
}

glm::vec3 MassPoint::getPosition() const {
    return this->_position;
}

glm::vec3 MassPoint::getVelocity() const {
    return this->_velocity;
}

glm::vec3 MassPoint::getForce() const {
    return this->_force;
}

float MassPoint::getMass() const {
    return this->_mass;
}

float MassPoint::getDamping() const {
    return this->_damping;
}

void MassPoint::setId(uint32_t id) {
    this->_id = id;
}

void MassPoint::setPosition(glm::vec3 position) {
    this->_position = position;
}

void MassPoint::setPosition(float x, float y, float z) {
    this->_position = glm::vec3(x, y, z);
}

void MassPoint::shiftPosition(glm::vec3 shift) {
    this->_position += shift;
}

void MassPoint::scalePosition(float scale) {
    this->_position *= scale;
}

void MassPoint::setVelocity(glm::vec3 velocity) {
    this->_velocity = velocity;
}

void MassPoint::setMass(float mass) {
    this->_mass = mass;
}

void MassPoint::setDamping(float damping) {
    this->_damping = damping;
}

void MassPoint::applyImpulse(glm::vec3 impulse) {
    this->_velocity += impulse / this->_mass;
}

void MassPoint::clearForce() {
    this->_force = glm::vec3(0.f);
}

void MassPoint::addForce(glm::vec3 force) {
    this->_force += force;
}

std::ostream& operator<<(std::ostream& os, const MassPoint& massPoint) {
    os << std::setprecision(3)
    << "Point Id:  " << massPoint._id << std::endl
    << "Point Position:  (" 
    << massPoint._position[0] << "; "
    << massPoint._position[1] << "; "
    << massPoint._position[2] << ")" << std::endl
    << "Point Velocity:  ("
    << massPoint._velocity[0] << "; "
    << massPoint._velocity[1] << "; "
    << massPoint._velocity[2] << ")" << std::endl
    << "Point Force:  ("
    << massPoint._force[0] << "; "
    << massPoint._force[1] << "; "
    << massPoint._force[2] << ")" << std::endl
    << "Point Mass:  " << massPoint._mass << std::endl
    << "Point Damping:  " << massPoint._damping;

    return os;
}

MassPoint& MassPoint::operator=(const MassPoint& other) {
    if (this == &other) return *this;

    this->_id = other._id;
    this->_position = other._position;
    this->_velocity = other._velocity;
    this->_force = other._force;
    this->_mass = other._mass;
    this->_damping = other._damping;

    return *this;
}

MassPoint& MassPoint::operator=(MassPoint&& other) noexcept {
    if (this == &other) return *this;

    this->_id = other._id;
    this->_position = std::move(other._position);
    this->_velocity = std::move(other._velocity);
    this->_force = std::move(other._force);
    this->_mass = other._mass;
    this->_damping = other._damping;

    return *this;
}

void MassPoint::onDraw(Renderer& renderer) {
    renderer.drawSphere(
        this->_position,
        0.01f * this->_mass,
        glm::vec4(0.15f, 0.15f, 1.f, 1.f)
    );
}