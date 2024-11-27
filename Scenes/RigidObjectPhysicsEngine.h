#pragma once
#include <vector>

#include "RigidObject3D.h"

class RigidObjectPhysicsEngine {
private:
    std::vector<RigidObject3D*> f_objects;

public:
    RigidObjectPhysicsEngine():
    f_objects()
    {}

    ~RigidObjectPhysicsEngine() {
        for (auto object: f_objects) delete object;
        f_objects.clear();
    }

    RigidObject3D getObject(uint32_t id) {
        return *f_objects.at(id);
    }

    void addObject(RigidObject3D* object) {
        f_objects.push_back(object);
    }

    void applyForceToObject(uint32_t id, const Force& force) {
        f_objects.at(id)->applyForce(force);
    }

    Force applyLocalForceToObject(uint32_t id, const Force& force) {
        return f_objects.at(id)->applyLocalForce(force);
    }

    void applyImpulseToObject(uint32_t id, const glm::vec3& impulse, const glm::vec3& applicationPoint) {
        f_objects.at(id)->applyImpulse(impulse, applicationPoint);
    }

    void euIntegrate(float delta) {
        for (auto object: f_objects) {
            object->euIntegrate(delta);
            object->flushForces();
        }
    }

    void mpIntegrate(float delta) {
        for (auto object: f_objects) {
            object->mpIntegrate(delta);
            object->flushForces();
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const RigidObjectPhysicsEngine& ROPE) {
		os << std::setprecision(3)
        << "    <^> Objects:   " << std::endl;
        uint32_t id = 0;
        for (auto object: ROPE.f_objects) {
            os << "Object #" << id << std::endl
            << *object << std::endl << std::endl;
            id++;
        }

		return os;
	}

    void onDraw(Renderer &renderer) {
        for (auto object: f_objects) {
            object->onDraw(renderer);
        }
    }
};