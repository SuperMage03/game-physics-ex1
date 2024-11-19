#pragma once
#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include "Point.h"

// Abstract class for Force Generators
class ForceGenerator {
public:
    virtual ~ForceGenerator();
    virtual void updateForce(Point& point, float duration) = 0;
};

class GravityForceGenerator : public ForceGenerator {
private:
    glm::vec3 gravity_;
public:
    GravityForceGenerator(const glm::vec3& gravity);
    void updateForce(Point& point, float duration) override;
};

class SpringForceGenerator : public ForceGenerator {
private:
    Point& other_;
    float stiffness_{0.0f};
    float initial_length_{0.0f};
public:
    SpringForceGenerator(Point& other, float stiffness, float initial_length);
    void updateForce(Point& point, float duration) override;
};

class DampingForceGenerator: public ForceGenerator {
public:
    void updateForce(Point& point, float duration) override;
};

// Singleton class for managing force generators
class ForceRegistry {
protected:
    ForceRegistry();
    static std::unique_ptr<ForceRegistry> singleton_;
    struct ForceRegistration {
        Point* point;
        ForceGenerator* force_generator;
        ForceRegistration(Point*const& point, ForceGenerator*const& force_generator);
    };
    std::vector<ForceRegistration> registry_;
public:
    static ForceRegistry* getInstance();

    // Singletons can't cloned nor copied
    ForceRegistry(const ForceRegistry&) = delete;
    ForceRegistry& operator=(const ForceRegistry&) = delete;
    
    // Methods
    void add(Point& point, ForceGenerator& force_generator);
    void remove(Point& point, ForceGenerator& force_generator);
    void clear();
    void updateForces(float duration=0.0f);
};
