#pragma once
#include <cstdint>
#include <iostream>
#include <iomanip>

class Spring {
private:
    uint32_t _id;
    uint32_t _point1;
    uint32_t _point2;
    float _stiffness;
    float _restLength;
    float _currentLength;

public:
    Spring():
    _id(0),
    _point1(-1),
    _point2(-1),
    _stiffness(1.),
    _restLength(1.),
    _currentLength(1.)
    {}

    Spring(const Spring& other);

    Spring(uint32_t id);

    Spring(uint32_t id, int point1, int point2, float stiffness, float restLength);

    uint32_t getId() const;

    uint32_t getPoint1() const;

    uint32_t getPoint2() const;

    float getStiffness() const;

    float getRestLength() const;

    float getCurrentLength() const;

    void setId(uint32_t id);

    void setPoint1(uint32_t point);

    void setPoint2(uint32_t point);

    void setPoints(uint32_t point1, uint32_t point2);

    void setStiffness(float stiffness);

    void setRestLength(float restLength);

    void setCurrentLength(float currentLength);

    friend std::ostream& operator<<(std::ostream& os, const Spring& spring);

    Spring& operator=(const Spring& other);

    Spring& operator=(Spring&& other);
};