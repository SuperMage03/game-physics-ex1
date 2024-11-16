#pragma once
#include "MassPoint.h"
#include "Spring.h"
#include "Environment.h"
#include <iostream>
#include <iomanip>

class MassSpringSystem {
private:
    unsigned _massPointCount;
    MassPoint* _massPoints;
    unsigned _springCount;
    Spring* _springs;
    Environment _environment;

public:
    MassSpringSystem():
    _massPointCount(0),
    _massPoints(nullptr),
    _springCount(0),
    _springs(nullptr),
    _environment()
    {}

    MassSpringSystem(const MassSpringSystem& other);

    MassSpringSystem(MassSpringSystem&& other);

    MassSpringSystem(unsigned massPointCount, unsigned springCount);

    MassSpringSystem(unsigned massPointCount, MassPoint* massPoints, unsigned springCount, Spring* springs);

    ~MassSpringSystem() {
        delete[] _massPoints;
        delete[] _springs;
    }

    unsigned getMassPointCount() const;

    MassPoint* getMassPoints() const;

    unsigned getSpringCount() const;

    Spring* getSprings() const;

    void setMassPoints(unsigned massPointCount, MassPoint* massPoints);

    void setSprings(unsigned springCount, Spring* springs);

    MassPoint* getMassPoint(uint32_t id);

    Spring* getSpring(uint32_t id);

    void setGravity(glm::vec3 gravity);

    void setDampingToAllMP(float damping);

    void setStiffnessToAllS(float stiffness);

private: 
    MassPoint* createMassPointArray(unsigned massPointCount);
    
    Spring* createSpringArray(unsigned springCount);

public:
    friend std::ostream& operator<<(std::ostream& os, const MassSpringSystem& MSS);

    MassSpringSystem& operator=(const MassSpringSystem& other);

    MassSpringSystem& operator=(MassSpringSystem&& other) noexcept;

    void applyElasticForces();

    void updateCurrentLengths();

    void clearForces();

    glm::vec3 calculateAcceleration(uint32_t mpId);

    void eulerIntegrate(float delta);

    void midpointIntegrate(float delta);

    void leapfrogIntegrate(float delta);

    void enforceSimpleCollision(MassPoint* massPoint);

    void enforceSimpleCollisionsToAllMP();

    void applyImpulseToAllMP(glm::vec3 impulse);

    void onDraw(Renderer& renderer);
};