#include "MassSpringSystem.h"

MassSpringSystem::MassSpringSystem(const MassSpringSystem& other) {
    this->_massPointCount = other._massPointCount;
    this->_massPoints = new MassPoint[other._massPointCount];
    this->_springCount = other._springCount;
    this->_springs = new Spring[other._springCount];

    for (int mpId = 0; mpId < other._massPointCount; mpId++) {
        this->_massPoints[mpId] = other._massPoints[mpId];
    }
    for (int sId = 0; sId < other._springCount; sId++) {
        this->_springs[sId] = other._springs[sId];
    }
}

MassSpringSystem::MassSpringSystem(MassSpringSystem&& other) {
    this->_massPointCount = other._massPointCount;
    this->_massPoints = other._massPoints;
    this->_springCount = other._springCount;
    this->_springs = other._springs;
}

MassSpringSystem::MassSpringSystem(unsigned massPointCount, unsigned springCount) {
    this->_massPointCount = massPointCount;
    this->_springCount = springCount;
    this->_massPoints = createMassPointArray(massPointCount);
    this->_springs = createSpringArray(springCount);
}

MassSpringSystem::MassSpringSystem(unsigned massPointCount, MassPoint* massPoints, unsigned springCount, Spring* springs) {
    this->_massPointCount = massPointCount;
    this->_massPoints = massPoints;
    this->_springCount = springCount;
    this->_springs = springs;
}

unsigned MassSpringSystem::getMassPointCount() const {
    return this->_massPointCount;
}

MassPoint* MassSpringSystem::getMassPoints() const {
    return this->_massPoints;
}

unsigned MassSpringSystem::getSpringCount() const {
    return this->_springCount;
}

Spring* MassSpringSystem::getSprings() const {
    return this->_springs;
}

void MassSpringSystem::setGravity(glm::vec3 gravity) {
    this->_environment.setGravity(gravity);
}

void MassSpringSystem::setDampingToAllMP(float damping) {
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        this->_massPoints[mpId].setDamping(damping);
    }
}

void MassSpringSystem::setStiffnessToAllS(float stiffness) {
    for (int sId = 0; sId < this->_springCount; sId++) {
        this->_springs[sId].setStiffness(stiffness);
    }
}

void MassSpringSystem::setMassPoints(unsigned massPointCount, MassPoint* massPoints) {
    delete[] this->_massPoints;
    this->_massPointCount = massPointCount;
    this->_massPoints = massPoints;
}

void MassSpringSystem::setSprings(unsigned springCount, Spring* springs) {
    delete[] this->_springs;
    this->_springCount = springCount;
    this->_springs = springs;
}

MassPoint* MassSpringSystem::getMassPoint(uint32_t id) {
    return this->_massPoints + id;
}

Spring* MassSpringSystem::getSpring(uint32_t id) {
    return this->_springs + id;
}

MassPoint* MassSpringSystem::createMassPointArray(unsigned massPointCount) {
    if (massPointCount == 0) return nullptr;
    MassPoint* massPointArray = new MassPoint[massPointCount];
    for (int mpId = 0; mpId < massPointCount; mpId++) {
        massPointArray[mpId].setId(mpId);
    }
    return massPointArray;
}
    
Spring* MassSpringSystem::createSpringArray(unsigned springCount) {
    if (springCount == 0) return nullptr;
    Spring* springArray = new Spring[springCount];
    for (int sId = 0; sId < springCount; sId++) {
        springArray[sId].setId(sId);
    }
    return springArray;
}

std::ostream& operator<<(std::ostream& os, const MassSpringSystem& MSS) {
    os
    << "Mass Point Count:  " << MSS._massPointCount << std::endl
    << "Mass Points:  " << std::endl << std::endl;
    for (int mpId = 0; mpId < MSS._massPointCount; mpId++) {
        os << MSS._massPoints[mpId] << std::endl << std::endl;
    }
    os << "Spring Count:  " << MSS._springCount << std::endl
    << "Springs:  " << std::endl << std::endl;
    for (int sId = 0; sId < MSS._springCount; sId++) {
        os << MSS._springs[sId] << std::endl << std::endl;
    }
    return os;
}

MassSpringSystem& MassSpringSystem::operator=(const MassSpringSystem& other) {
    if (this == &other) return *this;
    delete[] _massPoints;
    delete[] _springs;

    this->_massPointCount = other._massPointCount;
    this->_massPoints = new MassPoint[other._massPointCount];
    this->_springCount = other._springCount;
    this->_springs = new Spring[other._springCount];

    for (int mpId = 0; mpId < other._massPointCount; mpId++) {
        this->_massPoints[mpId] = other._massPoints[mpId];
    }
    for (int sId = 0; sId < other._springCount; sId++) {
        this->_springs[sId] = other._springs[sId];
    }

    return *this;
}

MassSpringSystem& MassSpringSystem::operator=(MassSpringSystem&& other) noexcept {
    if (this == &other) return *this;
    delete[] _massPoints;
    delete[] _springs;

    this->_massPointCount = other._massPointCount;
    this->_massPoints = other._massPoints;
    this->_springCount = other._springCount;
    this->_springs = other._springs;

    return *this;
}

void MassSpringSystem::applyElasticForces() {
    for (int sId = 0; sId < this->_springCount; sId++) {
        MassPoint* mp1 = getMassPoint(this->_springs[sId].getPoint1());
        MassPoint* mp2 = getMassPoint(this->_springs[sId].getPoint2());
        glm::vec3 force12 = 
            -this->_springs[sId].getStiffness()
            * (this->_springs[sId].getRestLength() - this->_springs[sId].getCurrentLength())
            * (mp2->getPosition() - mp1->getPosition())
            / this->_springs[sId].getCurrentLength();
        mp1->addForce(force12);
        mp2->addForce(-force12);
    }
}

void MassSpringSystem::updateCurrentLengths() {
    for (int sId = 0; sId < this->_springCount; sId++) {
        MassPoint* mp1 = getMassPoint(this->_springs[sId].getPoint1());
        MassPoint* mp2 = getMassPoint(this->_springs[sId].getPoint2());
        glm::vec3 distanceVec = mp1->getPosition() - mp2->getPosition();
        float currentLength = sqrt(distanceVec.x * distanceVec.x + distanceVec.y * distanceVec.y + distanceVec.z * distanceVec.z);
        this->_springs[sId].setCurrentLength(currentLength);
    }
}

void MassSpringSystem::clearForces() {
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        this->_massPoints[mpId].clearForce();
    }
}

glm::vec3 MassSpringSystem::calculateAcceleration(uint32_t mpId) {
    glm::vec3 acceleration = 
    this->_massPoints[mpId].getForce() / this->_massPoints[mpId].getMass() 
    + this->_environment.getGravity() 
    - this->_massPoints[mpId].getDamping() * this->_massPoints[mpId].getVelocity() / this->_massPoints[mpId].getMass();
    return acceleration;
}

void MassSpringSystem::eulerIntegrate(float delta) {
    clearForces();
    applyElasticForces();
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        MassPoint* massPoint = this->_massPoints + mpId;
        glm::vec3 newPos = massPoint->getPosition() + delta * massPoint->getVelocity();
        glm::vec3 newVel = massPoint->getVelocity() + delta * calculateAcceleration(mpId);
        massPoint->setPosition(newPos);
        massPoint->setVelocity(newVel);
    }
    enforceSimpleCollisionsToAllMP();
    updateCurrentLengths();
}

void MassSpringSystem::leapfrogIntegrate(float delta) {
    clearForces();
    applyElasticForces();
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        MassPoint* massPoint = this->_massPoints + mpId;
        glm::vec3 newVel = massPoint->getVelocity() + delta * calculateAcceleration(mpId);
        massPoint->setVelocity(newVel);
        glm::vec3 newPos = massPoint->getPosition() + delta * massPoint->getVelocity();
        massPoint->setPosition(newPos);
    }
    enforceSimpleCollisionsToAllMP();
    updateCurrentLengths();
}

void MassSpringSystem::midpointIntegrate(float delta) {
    clearForces();
    applyElasticForces();
    glm::vec3* positions = new glm::vec3[this->_massPointCount];
    glm::vec3* velocities = new glm::vec3[this->_massPointCount];
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        MassPoint* massPoint = this->_massPoints + mpId;
        positions[mpId] = massPoint->getPosition();
        velocities[mpId] = massPoint->getVelocity();
        glm::vec3 mpPos = massPoint->getPosition() + (delta / 2.f) * massPoint->getVelocity();
        glm::vec3 mpVel = massPoint->getVelocity() + (delta / 2.f) * calculateAcceleration(mpId);
        massPoint->setPosition(mpPos);
        massPoint->setVelocity(mpVel);
    }
    enforceSimpleCollisionsToAllMP();
    updateCurrentLengths();
    clearForces();
    applyElasticForces();
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        MassPoint* massPoint = this->_massPoints + mpId;
        glm::vec3 newPos = positions[mpId] + delta * massPoint->getVelocity();
        glm::vec3 newVel = velocities[mpId] + delta * calculateAcceleration(mpId);
        massPoint->setPosition(newPos);
        massPoint->setVelocity(newVel);
    }
    enforceSimpleCollisionsToAllMP();
    updateCurrentLengths();

    delete[] positions;
    delete[] velocities;
}



void MassSpringSystem::enforceSimpleCollision(MassPoint* massPoint) {
    float border = 2.5f;
    float conservation = 0.5f;
    glm::vec3 correctedPosition(massPoint->getPosition());
    glm::vec3 correctedVelocity(massPoint->getVelocity());
    if (massPoint->getPosition().z < -border && correctedVelocity.z < 0.f) {
        correctedPosition.z = -border;
        if (correctedVelocity.z < 0.f) correctedVelocity.z *= -conservation;
    }
    if (massPoint->getPosition().x < -border) {
        correctedPosition.x = -border;
        if (correctedVelocity.x < 0.f) correctedVelocity.x *= -conservation;
    }
    if (massPoint->getPosition().x > border) {
        correctedPosition.x = border;
        if (correctedVelocity.x > 0.f) correctedVelocity.x *= -conservation;
    }
    if (massPoint->getPosition().y < -border) {
        correctedPosition.y = -border;
        if (correctedVelocity.y < 0.f) correctedVelocity.y *= -conservation;
    }
    if (massPoint->getPosition().y > border) {
        correctedPosition.y = border;
        if (correctedVelocity.y > 0.f) correctedVelocity.y *= -conservation;
    }
    massPoint->setPosition(correctedPosition);
    massPoint->setVelocity(correctedVelocity);
}

void MassSpringSystem::enforceSimpleCollisionsToAllMP() {
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        MassPoint* massPoint = this->_massPoints + mpId;
        enforceSimpleCollision(massPoint);
    }
}

void MassSpringSystem::applyImpulseToAllMP(glm::vec3 impulse) {
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        this->_massPoints[mpId].applyImpulse(impulse);
    }
}

void MassSpringSystem::onDraw(Renderer& renderer) {
    for (int sId = 0; sId < this->_springCount; sId++) {
        Spring* spring = this->_springs + sId;
        renderer.drawLine(
            this->_massPoints[spring->getPoint1()].getPosition(),
            this->_massPoints[spring->getPoint2()].getPosition(),
            glm::vec4(0.8f, 0.8f, 0.8f, 1.f)
        );
    }
    for (int mpId = 0; mpId < this->_massPointCount; mpId++) {
        this->_massPoints[mpId].onDraw(renderer);
    }
}