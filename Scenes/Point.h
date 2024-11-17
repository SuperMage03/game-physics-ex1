#pragma once
#include <ostream>
#include <vector>
#include <memory>
#include <glm/glm.hpp>

// Class representing a point in space
class Point {
private:
    glm::vec3 position_{0.0f};
    glm::vec3 velocity_{0.0f};
    glm::vec3 force_{0.0f};
    float mass_{1.0f};
    float damping_{0.0f};
public:
    Point(float mass=1.0f);
    Point(glm::vec3 position, glm::vec3 velocity, glm::vec3 force, float mass, float damping);
    // Getters
    float getMass() const;
    glm::vec3 getPosition() const;
    glm::vec3 getVelocity() const;
    glm::vec3 getForce() const;
    glm::vec3 getAcceleration() const;
    // Setters
    void setPosition(const glm::vec3& position);
    void setVelocity(const glm::vec3& velocity);
    void setForce(const glm::vec3& force);
    void setAcceleration(const glm::vec3& acceleration);
    // Methods
    void addForce(const glm::vec3& force);

    // Operator Overloads
    friend std::ostream& operator<< (std::ostream& out_stream, const Point& point);
};

// Singleton class for managing force generators
class PointRegistry {
public:
    enum class IntegrationMode {
        EULER,
        MIDPOINT,
    };
protected:
    struct PointRegistration {
        Point* point;
        PointRegistration(Point*const& point);
    };
    static std::unique_ptr<PointRegistry> singleton_;
    
    std::vector<PointRegistration> registry_;
    IntegrationMode integration_mode_;

    PointRegistry();
    void simulateStepEuler(const float& step);
    void simulateStepMidpoint(const float& step);
public:
    static PointRegistry* getInstance();

    // Singletons can't cloned nor copied
    PointRegistry(const PointRegistry&) = delete;
    PointRegistry& operator=(const PointRegistry&) = delete;

    // Getters
    IntegrationMode getIntegrationMode() const;

    // Setters
    void setIntegrationMode(const IntegrationMode& integration_mode);

    // Methods
    void add(Point& point);
    void remove(Point& point);
    void clear();
    void clearForces();
    void simulateStep(const float& step=0.1f);
};
