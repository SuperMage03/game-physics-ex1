#pragma once
#include <vector>

#include "RigidObject3D.h"
#include "StaticObject3D.h"
#include "Collision.h"
#include "Grid.h"

namespace Physics {
    /** @brief
     * Kinematic integration type enumeration.
     */
    enum class IntegrationType {
        EULER,
        MIDPOINT
    };

    /**
     * Responsible for numerical integration of kinematic properties of the system
     * and handles collisions.
     */
    class RigidObjectPhysicsEngine {
    private:
        /// @brief Vector of shared pointers to rigid objects of the system
        std::vector<std::shared_ptr<RigidObject3D>> f_rigidObjects;
        /// @brief Vector of shared pointers to static objects of the system
        std::vector<std::shared_ptr<StaticObject3D>> f_staticObjects;
        /// @brief pointer to grid object of the system
        Grid::Grid2D* f_gridObject;
    public:
        /// @brief Integration type field
        IntegrationType f_integrationType;

    public:

        #pragma region Constructors/Destructors

        RigidObjectPhysicsEngine():
        f_rigidObjects(),
        f_staticObjects(),
        f_integrationType(IntegrationType::MIDPOINT)
        {}

        ~RigidObjectPhysicsEngine() {
            for (auto object: f_rigidObjects) object.reset();
            for (auto object: f_staticObjects) object.reset();
            f_rigidObjects.clear();
            f_staticObjects.clear();
        }

        #pragma endregion

        #pragma region Getters/Setters

        std::shared_ptr<RigidObject3D> getRigidObject(uint32_t id) {
            return f_rigidObjects.at(id);
        }

        void addRigidObject(RigidObject3D* object) {
            f_rigidObjects.push_back(std::shared_ptr<RigidObject3D>(object));
        }

        void addRigidObject(std::shared_ptr<RigidObject3D> object) {
            f_rigidObjects.push_back(object);
        }

        std::shared_ptr<StaticObject3D> getStaticObject(uint32_t id) {
            return f_staticObjects.at(id);
        }

        void addStaticObject(StaticObject3D* object) {
            f_staticObjects.push_back(std::shared_ptr<StaticObject3D>(object));
        }

        void addStaticObject(std::shared_ptr<StaticObject3D> object) {
            f_staticObjects.push_back(object);
        }

        void setGridObject(Grid::Grid2D* grid) {
            f_gridObject = grid;
        }

        #pragma endregion

        /// @brief Applies a force to the rigid object with given id.
        /// @param id Id of affected object.
        /// @param force Force to be applied.
        void applyForceToRigidObject(uint32_t id, const Force& force) {
            f_rigidObjects.at(id)->applyForce(force);
        }

        /// @brief Applies a local force to the rigid object with given id.
        /// @param id Id of affected object.
        /// @param force Local force to be applied.
        /// @return Parameter force transformed to global force.
        Force applyLocalForceToRigidObject(uint32_t id, const Force& force) {
            return f_rigidObjects.at(id)->applyLocalForce(force);
        }

        /// @brief Applies an impulse to to the rigid object with given id.
        /// @param id Id of affected object.
        /// @param impulse Impulse to be applied.
        /// @param applicationPoint Global point of application.
        void applyImpulseToRigidObject(uint32_t id, const glm::dvec3& impulse, const glm::dvec3& applicationPoint) {
            f_rigidObjects.at(id)->applyImpulse(impulse, applicationPoint);
        }

        /// @brief Performes an Euler integration step for the system kinematic properties.
        /// @param delta Time step.
        void euIntegrate(double delta) {
            for (auto object: f_rigidObjects) {
                object->euIntegrate(delta);
                object->flushForces();
            }
        }

        /// @brief Performes a midpoint integration step for the system kinematic properties.
        /// @param delta Time step.
        void mpIntegrate(double delta) {
            for (auto object: f_rigidObjects) {
                object->mpIntegrate(delta);
                object->flushForces();
            }
        }

        /// @brief Simulates a single time step of a kinematic system, performs integration and handles collisions.
        /// @param delta Time step.
        void simulateStep(double delta) {
            // Integrate
            switch (f_integrationType)
            {
            case IntegrationType::EULER:
                euIntegrate(delta);
                break;
            case IntegrationType::MIDPOINT:
                mpIntegrate(delta);
            default:
                break;
            }

            // TODO: Collision detection and handling
            std::vector<RigidObjectCollision> collisions;
            // Check for collisions between balls
            for (int idA = 0; idA < f_rigidObjects.size(); idA++) {
                for (int idB = idA + 1; idB < f_rigidObjects.size(); idB++) {
                    std::shared_ptr<RigidObject3D> objA = f_rigidObjects.at(idA);
                    std::shared_ptr<RigidObject3D> objB = f_rigidObjects.at(idB);
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // !!!!!!!!!!!!!! Cast to Ball !!!!!!!!!!!!!
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // Should be done by introducing collider classes
                    std::shared_ptr<RigidBall> ballA = std::dynamic_pointer_cast<RigidBall>(objA);
                    std::shared_ptr<RigidBall> ballB = std::dynamic_pointer_cast<RigidBall>(objB);
                    // Check if ball collide
                    CollisionInfo collInfo = ballB->collidesWithBallInfo(*ballA);
                    if (collInfo.f_normal != glm::dvec3(0.)) {
                        // Create collision
                        RigidObjectCollision collision(
                            ballA,
                            ballB,
                            collInfo.f_collisionPoint,
                            collInfo.f_normal,
                            collInfo.f_depth
                        );
                        collisions.emplace_back(std::move(collision));
                    }
                }
            }
            
            // Handle collisions
            for (const auto collision: collisions) {
                handleRigidBodyCollsion(collision);
            }

            // Hadle wall collisions
            for (const auto objA: f_rigidObjects) {
                for (const auto objB: f_staticObjects) {
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // !!!!!!!!!!!!!! Cast to Ball !!!!!!!!!!!!!
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // Should be done by introducing collider classes
                    std::shared_ptr<RigidBall> ballA = std::dynamic_pointer_cast<RigidBall>(objA);
                    std::shared_ptr<StaticCuboid> wallB = std::dynamic_pointer_cast<StaticCuboid>(objB);
                    // Get collision info
                    CollisionInfo collInfo = wallB->collidesWithBallInfo(ballA->f_transform.f_position, ballA->f_transform.f_scale.x);
                    if (collInfo.f_normal == glm::dvec3(0.)) continue;

                    // Collision point
                    glm::dvec3 collisionPoint =  collInfo.f_collisionPoint;
                    // Collision point local
                    // Relative velocity
                    glm::dvec3 relativeVelocity = ballA->getVelocityAtGlobalPoint(collisionPoint);
                    glm::dvec3 collisionPointALocal = collisionPoint - ballA->f_transform.f_position;
                    // Calculate impulse
                    double J = -(1. + ballA->f_c) * std::min(glm::dot(relativeVelocity, collInfo.f_normal), 0.)
                        / (
                            1. / ballA->f_mass
                            + glm::dot(glm::cross((ballA->f_inertiaTensorInv * glm::cross(collisionPointALocal, collInfo.f_normal)), collisionPointALocal), collInfo.f_normal)
                        );
                    // std::cout << collisionCornerAId << std::endl;
                    // std::cout << J << std::endl;
                    // Calculate tangent
                    glm::dvec3 tangentNonNormed = glm::cross(glm::cross(collInfo.f_normal, relativeVelocity), collInfo.f_normal);
                    glm::dvec3 tangent = glm::dvec3(0.);
                    if (glm::length(tangentNonNormed) != 0) {
                        tangent = glm::normalize(tangentNonNormed);
                    }
                    
                    // Update box velocity
                    ballA->f_velocity += 
                        J
                        * (collInfo.f_normal - ballA->f_mu * tangent) 
                        / ballA->f_mass;
                    // Update box angular momentum
                    ballA->f_angularMomentum += J * glm::cross(collisionPointALocal, collInfo.f_normal);
                    // Recalculate angular velocity
                    ballA->calculateAngularVelocity();

                    // Look at result velocity
                    glm::dvec3 resultVelocity = relativeVelocity;
                    
                    // Push out of the wall
                    ballA->f_transform.f_position += (collInfo.f_depth + 0.00001) * collInfo.f_normal;
                }
            }

            // Hadle grid point collisions
            for (const auto objA: f_rigidObjects) {
                std::shared_ptr<RigidBall> ballA = std::dynamic_pointer_cast<RigidBall>(objA);
                // for (unsigned i = 0; i < f_gridObject->getN(); i++) {
                //     for (unsigned j = 0; j < f_gridObject->getM(); j++) {
                //         // glm::dvec3 cur_gridpoint = f_gridObject->getPoint3D(i, j);
                //         // if (ballA->containsPoint(cur_gridpoint)) {
                //         //     std::cout << "Gamer" << std::endl;
                //         // }
                //     }
                // }
                // std::cout << f_gridObject->getGridPositionFromWorldPosition({-2.5, -2.5}).x << ", " << f_gridObject->getGridPositionFromWorldPosition({-2.5, -2.5}).y << std::endl;
            }
        }

        /// @brief Handles a single RigidObjectCollision.
        /// @param collision Rigid Object collision to handle.
        void handleRigidBodyCollsion(const RigidObjectCollision& collision) {
            // Handle object A
            // Update velocity
            collision.f_objA->f_velocity += 
                collision.f_impulse
                // Take average roughness to conserve momentum 
                * (collision.f_normal - (collision.f_objA->f_mu + collision.f_objB->f_mu) * collision.f_tangent / 2.) 
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
                * (collision.f_normal - (collision.f_objA->f_mu + collision.f_objB->f_mu) * collision.f_tangent / 2.) 
                / collision.f_objB->f_mass;
            // Update angular momentum
            collision.f_objB->f_angularMomentum -= glm::cross(collision.f_collisionPointBLocal, collision.f_impulse * collision.f_normal);
            // Recalculate angular velocity
            collision.f_objB->calculateAngularVelocity();

            //Push boxes out of eachother
            collision.f_objA->f_transform.f_position += collision.f_depth * collision.f_normal / 2.;
            collision.f_objB->f_transform.f_position -= collision.f_depth * collision.f_normal / 2.;
        }

        friend std::ostream& operator<<(std::ostream& os, const RigidObjectPhysicsEngine& ROPE) {
            os << std::setprecision(3)
            << "    <^> Rigid Objects:   " << std::endl;
            uint32_t id = 0;
            for (auto object: ROPE.f_rigidObjects) {
                os << "Object #" << id << std::endl
                << *object << std::endl << std::endl;
                id++;
            }
            os << "    <^> Static Objects:   " << std::endl;
            id = 0;
            for (auto object: ROPE.f_staticObjects) {
                os << "Object #" << id << std::endl
                << *object << std::endl << std::endl;
                id++;
            }

            return os;
        }

        void onDraw(Renderer &renderer) {
            for (auto object: f_staticObjects) {
                object->onDraw(renderer);
            }
            for (auto object: f_rigidObjects) {
                object->onDraw(renderer);
            }
        }
    };
}