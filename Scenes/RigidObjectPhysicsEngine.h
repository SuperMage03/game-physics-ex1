#pragma once
#include <vector>

#include "RigidObject3D.h"
#include "StaticObject3D.h"
#include "Collision.h"
#include "GridFunction.h"

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
        /// @brief pointer to grid function object of the system
        GridFunction::ScalarGridFunction2D* f_gridFunction;
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

        void setGridFunction(GridFunction::ScalarGridFunction2D* gridFunction) {
            f_gridFunction = gridFunction;
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
                    // Calculate tangent
                    glm::dvec3 tangentNonNormed = glm::cross(glm::cross(collInfo.f_normal, relativeVelocity), collInfo.f_normal);
                    glm::dvec3 tangent = glm::dvec3(0.);
                    if (glm::length(tangentNonNormed) != 0) {
                        tangent = glm::normalize(tangentNonNormed);
                    }
                    
                    // Update ball velocity
                    ballA->f_velocity += 
                        J
                        * (collInfo.f_normal - ballA->f_mu * tangent) 
                        / ballA->f_mass;
                    // Update ball angular momentum
                    ballA->f_angularMomentum += J * glm::cross(collisionPointALocal, collInfo.f_normal);
                    // Recalculate angular velocity
                    ballA->calculateAngularVelocity();
                    
                    // Push out of the wall
                    ballA->f_transform.f_position += (collInfo.f_depth + 0.00001) * collInfo.f_normal;
                }
            }

            // Hadle grid point collisions
            for (const auto objA: f_rigidObjects) {
                std::shared_ptr<RigidBall> ballA = std::dynamic_pointer_cast<RigidBall>(objA);
                // Get closest vertex
                glm::ivec2 vertexId = f_gridFunction->getGrid().getGridPositionFromWorldPosition(ballA->f_transform.f_position);
                // Get index radius to consider
                glm::ivec2 indexRadius = f_gridFunction->getGrid().getNeighbourhoodIndexRadius(ballA->f_transform.f_scale.x);

                // Initialize vectors for vertex collision data
                // Vector for colliding grid function vertices coordinates
                std::vector<glm::dvec3> collisionPoints;
                // Vector of collision depths for colliding grid function vertices
                std::vector<double> depths;
                // Vector of collision normals for colliding grid function vertices
                std::vector<glm::dvec3> normals;
                // Vector of collision relative velocities for colliding grid function vertices
                std::vector<glm::dvec3> relativeVelocities;
                // Vector of local (w.r.t. to ballA) collision points for colliding grid function vertices
                std::vector<glm::dvec3> localCollisionPoints;

                // Go over all vertices in the index radius
                unsigned iF = std::max(0, vertexId.x - indexRadius.x);
                unsigned jF = std::max(0, vertexId.y - indexRadius.y);
                unsigned iL = std::min(vertexId.x + indexRadius.x, int(f_gridFunction->getN() - 1));
                unsigned jL = std::min(vertexId.y + indexRadius.y, int(f_gridFunction->getM() - 1));
                for (unsigned i = iF; i <= iL; i++) {
                    for (unsigned j = jF; j <= jL; j++) {
                        // Get vertex coordinates
                        glm::dvec3 collisionPoint = f_gridFunction->getPoint3D(i, j);
                        // If ball contains vertex (there is a collision)
                        if (ballA->containsPoint(collisionPoint)) {
                            // Save colliding grid function vertex coordinates
                            collisionPoints.push_back(collisionPoint);
                            // Get normal to grid function at colliding vertex
                            glm::dvec3 normal = f_gridFunction->getNormal(i, j);
                            // Save normal
                            normals.push_back(normal);
                            // Get depth from cointainsPointD (returns distance to surface, so negative if point is inside)
                            double depth = -ballA->containsPointD(collisionPoint);
                            // Check if most of the ball is still above the surface
                            // This is true when the normal to the surface is oriented to the opposite
                            // direction compared to the push-out vector returned by pointDepthVector()
                            if (glm::dot(normal, ballA->pointDepthVector(collisionPoint)) < 0) {
                                // Then save the value returned by containsPointD()
                                depths.push_back(depth);
                            }
                            else {
                                // Otherwise the ball is more than halfway below the surface,
                                // thus the depth is not the shortest distance to the surface,
                                // but the distance to the diametrically opposed point
                                depths.push_back(2. * ballA->f_transform.f_scale.x - depth);
                            }
                            // Save relative velocity
                            relativeVelocities.push_back(ballA->getVelocityAtGlobalPoint(collisionPoint));
                            // Save local collision point
                            localCollisionPoints.push_back(collisionPoint - ballA->f_transform.f_position);
                        }
                    }
                }

                // If there is a colliding point
                if (!collisionPoints.empty()) {
                    std::cout << collisionPoints.size() << std::endl;

                    // Initialize final data
                    glm::dvec3 collisionPoint = glm::dvec3(0.);
                    glm::dvec3 depthVector = glm::dvec3(0.);
                    glm::dvec3 normal = glm::dvec3(0.);
                    glm::dvec3 relativeVelocity = glm::dvec3(0.);
                    glm::dvec3 localCollisionPoint = glm::dvec3(0.);

                    // Initialize variables for max depth search
                    double maxDepth = -1.;
                    unsigned maxDepthId = -1;
                    // Average/pick data over all collision points
                    unsigned vertexCtr = collisionPoints.size();
                    for (unsigned i = 0; i < vertexCtr; i++) {
                        // Average the collision point location
                        collisionPoint += collisionPoints.at(i) * (1. / vertexCtr);
                        // Find maximum depth
                        if (maxDepth < depths.at(i)) {
                            maxDepth = depths.at(i);
                            maxDepthId = i;
                        }
                        // Average the normal
                        normal += normals.at(i) * (1. / vertexCtr);
                        // Average the relative velocity
                        relativeVelocity += relativeVelocities.at(i) * (1. / vertexCtr);
                        // Average the local collision point
                        localCollisionPoint += localCollisionPoints.at(i) * (1. / vertexCtr);
                    }

                    // Infer the depth vector as maximum collision depth times the normal at the respective vertex
                    depthVector = maxDepth * normals.at(maxDepthId);

                    // Handle collision
                    // Calculate impulse
                    double J = -(1. + ballA->f_c) * std::min(glm::dot(relativeVelocity, normal), 0.)
                        / (
                            1. / ballA->f_mass
                            + glm::dot(glm::cross((ballA->f_inertiaTensorInv * glm::cross(localCollisionPoint, normal)), localCollisionPoint), normal)
                        );
                    // Calculate tangent
                    glm::dvec3 tangentNonNormed = glm::cross(glm::cross(normal, relativeVelocity), normal);
                    glm::dvec3 tangent = glm::dvec3(0.);
                    if (glm::length(tangentNonNormed) != 0) {
                        tangent = glm::normalize(tangentNonNormed);
                    }
                    
                    // Update ball velocity
                    ballA->f_velocity += 
                        J
                        * (normal - ballA->f_mu * tangent) 
                        / ballA->f_mass;
                    // Update ball angular momentum
                    ballA->f_angularMomentum += J * glm::cross(localCollisionPoint, normal);
                    // Recalculate angular velocity
                    ballA->calculateAngularVelocity();
                    
                    // Push out of the wall
                    ballA->f_transform.f_position += (1 + 0.00001) * depthVector;
                }
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