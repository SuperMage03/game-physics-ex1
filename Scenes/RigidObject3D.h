#pragma once

#include "Renderer.h"
#include "Transform.h"

/** @brief
 * 3D rigid object structure.
 * Encompases kinematic properties of a 3D object.
 */
struct RigidObject3D {
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
	f_torque(0.)
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
	f_torque(0.)
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
	f_torque(0.)
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
struct RigidBall : public RigidObject3D {
	//TODO: Write a Ball class with methods specific to given shape.
};
