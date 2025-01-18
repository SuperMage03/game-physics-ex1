#pragma once

#include "Renderer.h"
#include "RigidObject3D.h"

struct CollisionInfo {
	/// @brief Global collision point.
	glm::dvec3 f_collisionPoint;
	/// @brief Collision normal.
	glm::dvec3 f_normal;
	/// @brief Collision depth.
	double f_depth;

	CollisionInfo():
	f_collisionPoint(0.),
	f_normal(0.),
	f_depth(0.)
	{}
};


/** @brief
 * Rigid body collision structure.
 * Describes a collision between two RigidObject3D objects.
 */
struct RigidObjectCollision {
 	/// @brief A shared pointer to object A involved in the collision.
 	std::shared_ptr<RigidObject3D> f_objA;
    /// @brief A shared pointer to object B involved in the collision.
    std::shared_ptr<RigidObject3D> f_objB;
	/// @brief Global collision point.
	glm::dvec3 f_collisionPoint;
	/// @brief Collision point in object A local frame.
	glm::dvec3 f_collisionPointALocal;
    /// @brief Collision point in object B local frame.
	glm::dvec3 f_collisionPointBLocal;
	/// @brief Velocity of collision point of body A relative to collision point of body B.
	glm::dvec3 f_relativeVelocity;
	/// @brief Collision normal.
	glm::dvec3 f_normal;
	/// @brief Collision tangent.
	glm::dvec3 f_tangent;
	/// @brief Collision depth.
	double f_depth;
	/// @brief Collision impulse.
	double f_impulse;

    RigidObjectCollision():
    f_objA(nullptr),
    f_objB(nullptr),
	f_collisionPoint(0.),
	f_collisionPointALocal(0.),
	f_collisionPointBLocal(0.),
	f_relativeVelocity(0.),
	f_normal(0.),
	f_tangent(0.),
	f_impulse(0.),
	f_depth(0.)
    {}

	RigidObjectCollision(
		std::shared_ptr<RigidObject3D> objA, 
		std::shared_ptr<RigidObject3D> objB,
		glm::dvec3 collisionPoint,
		glm::dvec3 normal,
		double depth
	):
    f_objA(objA),
    f_objB(objB),
	f_collisionPoint(collisionPoint),
	f_normal(normal),
	f_depth(depth)
    {
		calculateRelativeVelocity();
		calculateTangent();
		calculateImpulse();
	}

    RigidObjectCollision(
		RigidObject3D* objA, 
		RigidObject3D* objB,
		glm::dvec3 collisionPoint,
		glm::dvec3 normal,
		double depth
	):
    f_objA(objA),
    f_objB(objB),
	f_collisionPoint(collisionPoint),
	f_normal(normal),
	f_depth(depth)
    {
		calculateRelativeVelocity();
		calculateTangent();
		calculateImpulse();
	}

private:
	/// @brief Calculates relative velocity of collision points f_relativeVelocity.
	void calculateRelativeVelocity() {
		// Calculate local collision point coordinates
		f_collisionPointALocal = f_collisionPoint - f_objA->f_transform.f_position;
		f_collisionPointBLocal = f_collisionPoint - f_objB->f_transform.f_position;

		// Calculate collision point velocities
		glm::dvec3 collPointVelocityA = f_objA->f_velocity + glm::cross(f_objA->f_angularVelocity, f_collisionPointALocal);
		glm::dvec3 collPointVelocityB = f_objB->f_velocity + glm::cross(f_objB->f_angularVelocity, f_collisionPointBLocal);

		// Calculate relative velocity
		f_relativeVelocity = collPointVelocityA - collPointVelocityB;
	}

	/// @brief Calculates collision tangent.
	void calculateTangent() {
		glm::dvec3 tangent = glm::cross(glm::cross(f_normal, f_relativeVelocity), f_normal);
		f_tangent = glm::dvec3(0.);
		if (glm::length(tangent) != 0) f_tangent = glm::normalize(tangent);
	}

	/// @brief Calculates collision impulse.
	void calculateImpulse() {
		f_impulse =
			-(1 + (f_objA->f_c + f_objB->f_c) / 2.) * std::min(glm::dot(f_relativeVelocity, f_normal), 0.)
			/ (
				1.f / f_objA->f_mass
				+ 1.f / f_objB->f_mass
				+ glm::dot(glm::cross((f_objA->f_inertiaTensorInv * glm::cross(f_collisionPointALocal, f_normal)), f_collisionPointALocal), f_normal)
				+ glm::dot(glm::cross((f_objB->f_inertiaTensorInv * glm::cross(f_collisionPointBLocal, f_normal)), f_collisionPointBLocal), f_normal)
			);
	}
};