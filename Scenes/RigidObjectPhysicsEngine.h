#pragma once
#include <vector>

#include "RigidObject3D.h"
#include "StaticObject3D.h"
#include "util/CollisionDetection.h"

class RigidObjectPhysicsEngine {
private:
    std::vector<RigidObject3D*> f_objects;
    std::vector<Wall> f_walls;
public:
    enum IntegrationTypes {
        EULER,
        MIDPOINT
    };

    int f_integrationType;

public:
    RigidObjectPhysicsEngine():
    f_objects(),
    f_walls(),
    f_integrationType(1)
    {}

    ~RigidObjectPhysicsEngine() {
        for (auto object: f_objects) delete object;
        f_objects.clear();
        f_walls.clear();
    }

    RigidObject3D getObject(uint32_t id) {
        return *f_objects.at(id);
    }

    void addObject(RigidObject3D* object) {
        f_objects.push_back(object);
    }

    Wall getWall(uint32_t id) {
        return f_walls.at(id);
    }

    void addWall(const Wall& wall) {
        f_walls.push_back(wall);
    }

    void addWall(Wall&& wall) {
        f_walls.emplace_back(std::move(wall));
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

    void simulateStep(float delta) {
        // Integrate
        switch (f_integrationType)
        {
        case EULER:
            euIntegrate(delta);
            break;
        case MIDPOINT:
            mpIntegrate(delta);
        default:
            break;
        }

        std::vector<Collision> collisions;
        // Check for collisions
        // checkCollisionSAT (Mat4& obj2World_A, Mat4& obj2World_B)
        for (int idA = 0; idA < f_objects.size(); idA++) {
            for (int idB = idA + 1; idB < f_objects.size(); idB++) {
                RigidObject3D* objA = f_objects.at(idA);
                RigidObject3D* objB = f_objects.at(idB);
                // Continue if same object
                // if (objA == objB) continue;
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // !!!!!!!!!!!!!! Cast to Box !!!!!!!!!!!!!!
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // Should be done by introducing collider classes
                Box* boxA = dynamic_cast<Box*>(objA);
                Box* boxB = dynamic_cast<Box*>(objB);
                // Get collision point of object A if exists
                glm::mat4 transformA = objA->f_transform.getTransform();
                glm::mat4 transformB = objB->f_transform.getTransform();
                auto collisionInfo = collisionTools::checkCollisionSAT(transformA, transformB);

                if (!collisionInfo.isColliding) continue;
                
                // Create collision
                Collision collision(
                    boxA,
                    boxB,
                    collisionInfo.collisionPointWorld,
                    collisionInfo.normalWorld
                );
                collisions.emplace_back(std::move(collision));
            }
        }
        
        // Handle collisions
        for (const auto collision: collisions) {
            handleCollsion(collision);
        }

        // Hadle wall collisions
        for (const auto objA: f_objects) {
            for (const auto wall: f_walls) {
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // !!!!!!!!!!!!!! Cast to Box !!!!!!!!!!!!!!
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // Should be done by introducing collider classes
                Box* boxA = dynamic_cast<Box*>(objA);
                // Get collision point of object A if exists
                int collisionCornerAId = wall.pointInside(boxA->getCornerPoints());
                if (collisionCornerAId == -1) continue;
                // Collision point
                glm::vec3 collisionPoint =  boxA->getCornerPointId(collisionCornerAId);
                // Collision point local
                // Relative velocity
                glm::vec3 relativeVelocity = boxA->getCornerPointVelocityId(collisionCornerAId);
                glm::vec3 collisionPointALocal = boxA->getCornerPointId(collisionCornerAId) - boxA->f_transform.f_position;
                // Calculate impulse
                float J = -(1.f + boxA->f_c) * std::min(glm::dot(relativeVelocity, wall.f_normal), 0.f)
                    / (
                        1.f / boxA->f_mass
                        + glm::dot(glm::cross((boxA->f_inertiaTensorInv * glm::cross(collisionPointALocal, wall.f_normal)), collisionPointALocal), wall.f_normal)
                    );
                // std::cout << collisionCornerAId << std::endl;
                // std::cout << J << std::endl;
                // Calculate tangent
                glm::vec3 tangentNonNormed = glm::cross(glm::cross(wall.f_normal, relativeVelocity), wall.f_normal);
                glm::vec3 tangent = glm::vec3(0.f);
                if (glm::length(tangentNonNormed) != 0) {
                    tangent = glm::normalize(tangentNonNormed);
                }
                
                // Update box velocity
                boxA->f_velocity += 
                    J
                    * (wall.f_normal - boxA->f_mu * tangent) 
                    / boxA->f_mass;
                // Update box angular momentum
                boxA->f_angularMomentum += J * glm::cross(collisionPointALocal, wall.f_normal);
                // Recalculate angular velocity
                boxA->calculateAngularVelocity();

                // Look at result velocity
                glm::vec3 resultVelocity = boxA->getCornerPointVelocityId(collisionCornerAId);
                
                // Push out of the wall
                boxA->f_transform.f_position += (wall.f_surface - glm::dot(collisionPoint, wall.f_normal) + 0.00001f) * wall.f_normal;
            }
        }

        
    }

	void handleCollsion(const Collision& collision) {
        // Handle object A
        // Update velocity
        collision.f_objA->f_velocity += 
            collision.f_impulse
            // Take average roughness to conserve momentum 
            * (collision.f_normal - (collision.f_objA->f_mu + collision.f_objB->f_mu) * collision.f_tangent / 2.f) 
            / collision.f_objA->f_mass;
        // Update angular momentum
        collision.f_objA->f_angularMomentum += glm::cross(collision.f_collisionPointALocal, collision.f_impulse * collision.f_normal);
        // Recalculate angular velocity
        collision.f_objA->calculateAngularVelocity();

        // Handle object B
        // Update velocity
        collision.f_objB->f_velocity -= 
            collision.f_impulse
            // Take average roughness to conserve momentum 
            * (collision.f_normal - (collision.f_objA->f_mu + collision.f_objB->f_mu) * collision.f_tangent / 2.f) 
            / collision.f_objB->f_mass;
        // Update angular momentum
        collision.f_objB->f_angularMomentum -= glm::cross(collision.f_collisionPointBLocal, collision.f_impulse * collision.f_normal);
        // Recalculate angular velocity
        collision.f_objB->calculateAngularVelocity();
	}

    void handleWallCollsion(RigidObject3D* object) {
        
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