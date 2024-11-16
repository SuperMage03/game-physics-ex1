#include "Spring.h"

Spring::Spring(const Spring& other) {
    this->_id = other._id;
    this->_point1 = other._point1;
    this->_point2 = other._point2;
    this->_stiffness = other._stiffness;
    this->_restLength = other._restLength;
    this->_currentLength = other._currentLength;
}

Spring::Spring(uint32_t id): Spring() {
    this->_id = id;
}

Spring::Spring(uint32_t id, int point1, int point2, float stiffness, float restLength): Spring(id) {
    this->_point1 = point1;
    this->_point2 = point2;
    this->_stiffness = stiffness;
    this->_restLength = restLength;
}

uint32_t Spring::getId() const {
    return this->_id;
}

uint32_t Spring::getPoint1() const {
    return this->_point1;
}

uint32_t Spring::getPoint2() const {
    return this->_point2;
}

float Spring::getStiffness() const {
    return this->_stiffness;
}

float Spring::getRestLength() const {
    return this->_restLength;
}

float Spring::getCurrentLength() const {
    return this->_currentLength;
}

void Spring::setId(uint32_t id) {
    this->_id = id;
}

void Spring::setPoint1(uint32_t point) {
    this->_point1 = point;
}

void Spring::setPoint2(uint32_t point) {
    this->_point2 = point;
}

void Spring::setPoints(uint32_t point1, uint32_t point2) {
    this->_point1 = point1;
    this->_point2 = point2;
}

void Spring::setStiffness(float stiffness) {
    this->_stiffness = stiffness;
}

void Spring::setRestLength(float restLength) {
    this->_restLength = restLength;
}

std::ostream& operator<<(std::ostream& os, const Spring& spring) {
    os << std::setprecision(3)
    << "Spring Id:  " << spring._id << std::endl
    << "Point 1 Id:  " << spring._point1 << std::endl
    << "Point 2 Id:  " << spring._point2 << std::endl
    << "Spring Stiffness:  " << spring._stiffness << std::endl
    << "Spring Rest Length:  " << spring._restLength << std::endl
    << "Spring Current Length:  " << spring._currentLength;

    return os;
}

Spring& Spring::operator=(const Spring& other) {
    if (this == &other) return *this;
    
    this->_id = other._id;
    this->_point1 = other._point1;
    this->_point2 = other._point2;
    this->_stiffness = other._stiffness;
    this->_restLength = other._restLength;
    this->_currentLength = other._currentLength;

    return *this;
}

Spring& Spring::operator=(Spring&& other) {
    if (this == &other) return *this;
    
    this->_id = other._id;
    this->_point1 = other._point1;
    this->_point2 = other._point2;
    this->_stiffness = other._stiffness;
    this->_restLength = other._restLength;
    this->_currentLength = other._currentLength;

    return *this;
}

void Spring::setCurrentLength(float currentLength) {
    this->_currentLength = currentLength;
}