#pragma once

#include "Renderer.h"
#include "Transform.h"
#include "Collision.h"

/** @brief
 * 3D rigid object structure.
 * Encompases kinematic properties of a 3D object.
 */
class RigidObject3D {
public:
	/// @brief Object mass
	double f_mass;
	/// @brief Current Inertia tensor
	glm::dmat3 f_inertiaTensorInv;
	/// @brief Elasticity/Plasticity
	double f_c;
	/// @brief Coefficient of friction
	double f_mu;

	/// @brief Object transform
	Transform3D f_transform;
	/// @brief Object velocity
	glm::dvec3 f_velocity;
	/// @brief Angular momentum
	glm::dvec3 f_angularMomentum;
	/// @brief Angular velocity
	glm::dvec3 f_angularVelocity;

	/// @brief Central force applied to the object
	glm::dvec3 f_centralForce;
	/// @brief Torque applied to the object
	glm::dvec3 f_torque;

	/// @brief Object color
	glm::vec4 f_color;

	#pragma region Constructors

	RigidObject3D():
	f_mass(1.),
	f_inertiaTensorInv(0.),
	f_c(0.),
	f_mu(0.),

	f_transform(),
	f_velocity(0.),
	f_angularMomentum(0.),
	f_angularVelocity(0.),

	f_centralForce(0.),
	f_torque(0.),

	f_color(1.)
	{}

	RigidObject3D(
		double mass,
		double c,
		double mu,
		const Transform3D& transform,
		const glm::dvec3& velocity,
		const glm::dvec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.),
	f_c(c),
	f_mu(mu),

	f_transform(transform),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.),
	f_torque(0.),

	f_color(1.)
	{}

	RigidObject3D(
		double mass,
		double c,
		double mu,
		Transform3D&& transform,
		glm::dvec3&& velocity,
		glm::dvec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.),
	f_c(c),
	f_mu(mu),

	f_transform(std::move(transform)),
	f_velocity(std::move(velocity)),
	f_angularVelocity(std::move(angularVelocity)),

	f_centralForce(0.),
	f_torque(0.),

	f_color(1.)
	{}

	#pragma endregion Constructors

	/// @brief Sets the force vector acting on the object to zero.
	void flushForces() {
		f_centralForce = glm::dvec3(0.);
		f_torque = glm::dvec3(0.);
	}

	/**
	 * @brief Calculates current inertia tensor inverse f_inertiaTensorInv 
	 * using rotation matrix and initial inertia tensor inverse.
	 */
	virtual void calculateInertiaTensorInv() {
		glm::dmat3 rotationMatrix = f_transform.getRotationMatrix();
		glm::dmat3 iiti = getInitialInertiaTensorInv();
		f_inertiaTensorInv = rotationMatrix * iiti * glm::transpose(rotationMatrix);
	}

	/// @brief Calculates initial inertia tensor inverse.
	/// @return glm::dmat3 initial inertia tensor inverse.
	virtual glm::dmat3 getInitialInertiaTensorInv() const {
		return glm::identity<glm::dmat3>();
	}

	/** @brief Calculates current angular momentum f_angularMomentum using
	 * current inertia tensor and angular velocity
	 */
	void calculateAngularMomentum() {
		f_angularMomentum = glm::inverse(f_inertiaTensorInv) * f_angularVelocity;
	}

	/** @brief Calculates current angular velocity f_angularVelocity using
	 * current inertia tensor inverse and angular momentum
	 */
	void calculateAngularVelocity() {
		f_angularVelocity = f_inertiaTensorInv * f_angularMomentum;
	}

	/// @brief Applies a global force to the object, both central and torque.
	void applyForce(const Force& force) {
		f_centralForce += force.f_force;
		f_torque += glm::cross(force.f_applicationPoint - f_transform.f_position, force.f_force);
	}

	/// @brief Applies a local force to the object, both central and torque.
	/// @param force Local force to be applied.
	/// @return Parameter force transformed to global force.
	Force applyLocalForce(const Force& force) {
		Force globalForce = f_transform.transformLocalForceToGlobalForce(force);
		applyForce(globalForce);
		return globalForce;
	}

	/// @brief Applies an impulse to the object at a given application point, both central and rotational.
	/// @param impulse Applied impulse vector.
	/// @param applicationPoint Application point coordinates.
	void applyImpulse(const glm::dvec3& impulse, const glm::dvec3& applicationPoint) {
		glm::dvec3 centralDirection = glm::normalize(applicationPoint - f_transform.f_position);
		glm::dvec3 centralImpulse = glm::dot(centralDirection, impulse) * centralDirection;
		glm::dvec3 rotationalImpulse = impulse - centralImpulse;

		f_velocity += centralImpulse / f_mass;
		f_angularMomentum += f_inertiaTensorInv * rotationalImpulse;
		calculateAngularVelocity();
	}

	/// @brief Performes an Euler integration step for the object kinematic properties.
	/// @param delta Time step.
	void euIntegrate(double delta) {
		// Integrate position
		f_transform.f_position += delta * f_velocity;
		// Integrate linear velocity
		f_velocity += delta * f_centralForce / f_mass;

		// Integrate rotation
		f_transform.f_quat += (delta / 2.) 
			* glm::dquat(0., f_angularVelocity[0], f_angularVelocity[1], f_angularVelocity[2])
			* f_transform.f_quat;
		// Normalize quaternion
		f_transform.f_quat = glm::normalize(f_transform.f_quat);
		// Integrate angular momentum
		f_angularMomentum += delta * f_torque;
		// Update inertia tensor
		calculateInertiaTensorInv();
		// Calculate angular velocity
		calculateAngularVelocity();
	}

	/// @brief Performes a midpoint integration step for the object kinematic properties.
	/// @param delta Time step.
	void mpIntegrate(double delta) {
		// Leapfrog for linear movement
		// Set midpoint value for linear velocity
		f_velocity += (delta / 2.) * f_centralForce / f_mass;
		// Integrate position
		f_transform.f_position += delta * f_velocity;
		// Complete linear velocity integration
		f_velocity += (delta / 2.) * f_centralForce / f_mass;

		// Integrate rotation
		// Save initial quat
		glm::dquat initialQuat = f_transform.f_quat;

		// Set midpoint values
		f_transform.f_quat += (delta / 4.) 
			* glm::dquat(0., f_angularVelocity[0], f_angularVelocity[1], f_angularVelocity[2])
			* f_transform.f_quat;
		// Normalize quaternion
		f_transform.f_quat = glm::normalize(f_transform.f_quat);
		// Set midpoint angular momentum
		f_angularMomentum += delta * f_torque / 2.;
		// Set midpoint inertia tensor
		calculateInertiaTensorInv();
		// Set midpoint angular velocity
		calculateAngularVelocity();

		// MP integrate
		f_transform.f_quat = initialQuat + (delta / 2.) 
			* glm::dquat(0., f_angularVelocity[0], f_angularVelocity[1], f_angularVelocity[2])
			* f_transform.f_quat;
		// Normalize quaternion
		f_transform.f_quat = glm::normalize(f_transform.f_quat);
		// Integrate angular momentum
		f_angularMomentum += delta * f_torque / 2.;
		// Update inertia tensor
		calculateInertiaTensorInv();
		// Calculate angular velocity
		calculateAngularVelocity();

	}

	friend std::ostream& operator<<(std::ostream& os, const RigidObject3D& object) {
		os << std::setprecision(3)
		<< "        <^> Elasticity:   "
		<< object.f_c << std::endl
		<< "        <^> Friction coefficient:   "
		<< object.f_mu << std::endl
		<< "        <^> Mass:   "
		<< object.f_mass << std::endl
		<< "        <^> Transform:   " << std::endl
		<< object.f_transform << std::endl
		<< "        <^> Velocity:   ("
		<< object.f_velocity[0] << "; "
		<< object.f_velocity[1] << "; "
		<< object.f_velocity[2] << ")" << std::endl
		<< "        <^> Angular momentum:   ("
		<< object.f_angularMomentum[0] << "; "
		<< object.f_angularMomentum[1] << "; "
		<< object.f_angularMomentum[2] << ")" << std::endl
		<< "        <^> Angular velocity:   ("
		<< object.f_angularVelocity[0] << "; "
		<< object.f_angularVelocity[1] << "; "
		<< object.f_angularVelocity[2] << ")";

		return os;
	}

	virtual void onDraw(Renderer &renderer) {};
};

/** @brief
 * 3D rigid ball structure.
 * Encompases kinematic properties and shape properties.
 */
class RigidBall : public RigidObject3D {
public:
	#pragma region Constructors

	RigidBall(
		double mass,
		double c,
		double mu,
		const Transform3D& transform,
		const glm::dvec3& velocity,
		const glm::dvec3& angularVelocity
	): RigidObject3D (
		mass,
		c,
		mu,
		transform,
		velocity,
		angularVelocity
	)
	{}

	RigidBall(
		double mass,
		double c,
		double mu,
		Transform3D&& transform,
		glm::dvec3&& velocity,
		glm::dvec3&& angularVelocity
	): RigidObject3D (
		mass,
		c,
		mu,
		std::move(transform),
		std::move(velocity),
		std::move(angularVelocity)
	)
	{}

	#pragma endregion

	/// @brief Determines whether the ball contains the point.
	/// @param point Point to check.
	/// @return Returns true if the point is inside the ball or on the surface, returns false otherwise.
	bool containsPoint(const glm::dvec3& point) const {
		return glm::length(point - f_transform.f_position) <= f_transform.f_scale.x;
	}
	
	/// @brief Determines whether the point is inside the ball.
	/// @param point Point to check.
	/// @return Returns double distance to the ball surface (negative if point is inside of the ball).
	double containsPointD(const glm::dvec3& point) const {
		return glm::length(point - f_transform.f_position) - f_transform.f_scale.x;
	}

	/// @brief Calculates a push-out vector of magnitude of depth.
	/// @param point Point to check.
	/// @return Returns glm::dvec3 push-out vector.
	glm::dvec3 pointDepthVector(const glm::dvec3& point) const {
		return glm::normalize(f_transform.f_position - point) * (glm::length(point - f_transform.f_position) - f_transform.f_scale.x);
	}

	/// @brief Determines whether the ball collides with another ball.
	/// @param other Ball to check collision with.
	/// @return Returns true if the balls collide, returns false otherwise.
	bool collidesWithBall(const RigidBall& other) const {
		return glm::length(other.f_transform.f_position - f_transform.f_position) <= (other.f_transform.f_scale.x + f_transform.f_scale.x);
	}

	/// @brief Determines whether the ball collides with another ball and returns a collision info structure.
    /// @param other Ball to check collision with.
    /// @return Returns the collision info strucutre (empty if no collision).
	CollisionInfo collidesWithBallInfo(const RigidBall& other) const {
		CollisionInfo collInfo;
		float distance = glm::length(other.f_transform.f_position - f_transform.f_position);

		if (distance <= (other.f_transform.f_scale.x + f_transform.f_scale.x)) {
			collInfo.f_normal = glm::normalize(other.f_transform.f_position - f_transform.f_position);
			collInfo.f_depth = (other.f_transform.f_scale.x + f_transform.f_scale.x) - distance;
			collInfo.f_collisionPoint = f_transform.f_position + collInfo.f_normal * (f_transform.f_scale.x + collInfo.f_depth / 2.);
		}

		return collInfo;
	}

	/// @brief Calculates velocity of a given point, if it was part of this ball.
	/// @param point Point to calculate velocity at.
	/// @return glm::dvec3 velocity vector at given point
	glm::dvec3 getVelocityAtGlobalPoint(glm::dvec3 point) const {
		return f_velocity + glm::cross(f_angularVelocity, point - f_transform.f_position);
	}

	/// @brief Calculates initial inertia tensor inverse.
	/// @return glm::dmat3 initial inertia tensor inverse.
	glm::dmat3 getInitialInertiaTensorInv() const override {
		return glm::dmat3 (
			5. / (2. * f_mass * f_transform.f_scale.x * f_transform.f_scale.x), 0., 0.,
			0., 5. / (2. * f_mass * f_transform.f_scale.x * f_transform.f_scale.x), 0.,
			0., 0., 5. / (2. * f_mass * f_transform.f_scale.x * f_transform.f_scale.x)
		);
	}
	
	void onDraw(Renderer &renderer) override {
		renderer.drawSphere(
			f_transform.f_position,
			f_transform.f_scale.x,
			f_color
		);
	}
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