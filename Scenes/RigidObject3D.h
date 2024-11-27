#pragma once
#include "Renderer.h"
#include <iostream>
#include <glm/gtx/quaternion.hpp>

struct Force {
	// Force vector
	glm::vec3 f_force;
	// Force application point
	glm::vec3 f_applicationPoint;

	Force():
	f_force(0.f),
	f_applicationPoint(0.f)
	{}

	Force(
		const glm::vec3& force,
		const glm::vec3& applicationPoint
	):
	f_force(force),
	f_applicationPoint(applicationPoint)
	{}

	Force(
		glm::vec3&& force,
		glm::vec3&& applicationPoint
	):
	f_force(std::move(force)),
	f_applicationPoint(std::move(applicationPoint))
	{}


	void onDraw(Renderer &renderer) {
		renderer.drawLine(
			f_applicationPoint,
			f_applicationPoint + f_force,
			glm::vec3(0.15f, 0.15f, 1.f)
		);
		renderer.drawSphere(
			f_applicationPoint + f_force,
			0.01f,
			glm::vec4(0.15f, 0.15f, 1.f, 1.f)
		);
	}

	friend std::ostream& operator<<(std::ostream& os, const Force& force) {
		os << std::setprecision(3)
		<< "    <^> Force vector:   ("
		<< force.f_force[0] << "; "
		<< force.f_force[1] << "; "
		<< force.f_force[2] << ")" << std::endl
		<< "    <^> Force application point:   ("
		<< force.f_applicationPoint[0] << "; "
		<< force.f_applicationPoint[1] << "; "
		<< force.f_applicationPoint[2] << ")";

		return os;
	}
};

struct Transform3D {
	// Object center of mass position, [x, y, z]
	glm::vec3 f_position;
	// Object scale, [s_x, s_y, s_z]
	glm::vec3 f_scale;
	// Object rotation, quaternion
	glm::quat f_quat;

	Transform3D():
	f_position(0.f),
	f_scale(1.f),
	f_quat(glm::vec3(0.f))
	{}

	Transform3D(
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::quat& quat
	):
	f_position(position),
	f_scale(scale),
	f_quat(quat)
	{}

	Transform3D(
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::vec3& angles
	):
	f_position(position),
	f_scale(scale),
	f_quat(angles)
	{}

	glm::mat3 getRotationMatrix() const {
		return glm::toMat3(f_quat);
	}

	glm::mat4 getTransform() const {
		glm::mat4 transform = glm::mat4_cast(f_quat);
		transform[0][0] = f_scale[0];
		transform[1][1] = f_scale[1];
		transform[2][2] = f_scale[2];

		transform[0][3] = f_position[0];
		transform[1][3] = f_position[0];
		transform[2][3] = f_position[0];
		transform[3][3] = 1.f;
		return transform;
	}

	friend std::ostream& operator<<(std::ostream& os, const Transform3D& transform) {
		os << std::setprecision(3)
		<< "    <^> Position:   ("
		<< transform.f_position[0] << "; "
		<< transform.f_position[1] << "; "
		<< transform.f_position[2] << ")" << std::endl
		<< "    <^> Scale:   ("
		<< transform.f_scale[0] << "; "
		<< transform.f_scale[1] << "; "
		<< transform.f_scale[2] << ")" << std::endl
		<< "    <^> Quaternion:   ("
		<< transform.f_quat[0] << "; "
		<< transform.f_quat[1] << "; "
		<< transform.f_quat[2] << "; "
		<< transform.f_quat[3] << ")";

		return os;
	}

	glm::vec3 transformLocalToGlobal(const glm::vec3 position) {
		return getRotationMatrix() * position + f_position;
	}

	glm::vec3 transformGlobalToLocal(const glm::vec3 position) {
		return glm::inverse(getRotationMatrix()) * position - glm::inverse(getRotationMatrix()) * f_position;
	}

	Force transformLocalForceToGlobalForce(const Force& force) {
		glm::vec3 globalApplicationPoint = transformLocalToGlobal(force.f_applicationPoint);
		glm::vec3 globalForce = getRotationMatrix() * force.f_force;
		return Force(globalForce, globalApplicationPoint);
	}

	Force transformGlobalForceToLocalForce(const Force& force) {
		glm::vec3 localApplicationPoint = transformGlobalToLocal(force.f_applicationPoint);
		glm::vec3 localForce = glm::inverse(getRotationMatrix()) * force.f_force;
		return Force(localForce, localApplicationPoint);
	}
};

struct RigidObject3D {
	// Object mass
	float f_mass;
	// Current Inertia tensor
	glm::mat3 f_inertiaTensorInv;

	// Object transform
	Transform3D f_transform;
	// Object velocity
	glm::vec3 f_velocity;
	// Angular momentum
	glm::vec3 f_angularMomentum;
	// Angular velocity
	glm::vec3 f_angularVelocity;

	// Central force applied to the object
	glm::vec3 f_centralForce;
	// Torque applied to the object
	glm::vec3 f_torque;


	#pragma region Constructors
	RigidObject3D():
	f_mass(1.f),
	f_inertiaTensorInv(0.f),

	f_transform(),
	f_velocity(0.f),
	f_angularMomentum(0.f),
	f_angularVelocity(0.f),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::quat& quat,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),

	f_transform(position, scale, quat),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::vec3& angles,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),

	f_transform(position, scale, angles),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::quat&& quat,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),

	f_transform(std::move(position), std::move(scale), std::move(quat)),
	f_velocity(std::move(velocity)),
	f_angularVelocity(std::move(angularVelocity)),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::vec3&& angles,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),

	f_transform(std::move(position), std::move(scale), std::move(angles)),
	f_velocity(std::move(velocity)),
	f_angularVelocity(std::move(angularVelocity)),

	f_centralForce(0.f),
	f_torque(0.f)
	{}
	#pragma endregion Constructors

	void flushForces() {
		f_centralForce = glm::vec3(0.f);
		f_torque = glm::vec3(0.f);
	}

	virtual void calculateInertiaTensorInv() {
		glm::mat3 rotationMatrix = f_transform.getRotationMatrix();
		glm::mat3 iiti = getInitialInertiaTensorInv();
		f_inertiaTensorInv = rotationMatrix * iiti * glm::transpose(rotationMatrix);
	}

	virtual glm::mat3 getInitialInertiaTensorInv() const {
		return glm::identity<glm::mat3>();
	}

	void calculateAngularMomentum() {
		f_angularMomentum = glm::inverse(f_inertiaTensorInv) * f_angularVelocity;
	}

	void calculateAngularVelocity() {
		f_angularVelocity = f_inertiaTensorInv * f_angularMomentum;
	}

	void applyForce(const Force& force) {
		f_centralForce += force.f_force;
		f_torque += glm::cross(force.f_applicationPoint - f_transform.f_position, force.f_force);
	}

	Force applyLocalForce(const Force& force) {
		Force globalForce = f_transform.transformLocalForceToGlobalForce(force);
		applyForce(globalForce);
		return globalForce;
	}

	void applyImpulse(const glm::vec3& impulse, const glm::vec3& applicationPoint) {
		glm::vec3 centralDirection = glm::normalize(applicationPoint - f_transform.f_position);
		glm::vec3 centralImpulse = glm::dot(centralDirection, impulse) * centralDirection;
		glm::vec3 rotationalImpulse = impulse - centralImpulse;

		f_velocity += centralImpulse / f_mass;
		f_angularMomentum += f_inertiaTensorInv * rotationalImpulse;
		calculateAngularVelocity();
	}

	void euIntegrate(float delta) {
		// Integrate linear velocity
		f_velocity += delta * f_centralForce / f_mass;
		// Integrate position
		f_transform.f_position += delta * f_velocity;

		// Integrate rotation
		f_transform.f_quat += (delta / 2.f) 
			* glm::quat(0.f, f_angularVelocity[0], f_angularVelocity[1], f_angularVelocity[2])
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

	void mpIntegrate(float delta) {
		// Integrate linear velocity
		f_velocity += delta * f_centralForce / f_mass;
		// Integrate position
		f_transform.f_position += delta * f_velocity;

		// Integrate rotation
		// Save initial quat
		glm::quat initialQuat = f_transform.f_quat;

		// Set midpoint values
		f_transform.f_quat += (delta / 4.f) 
			* glm::quat(0.f, f_angularVelocity[0], f_angularVelocity[1], f_angularVelocity[2])
			* f_transform.f_quat;
		// Normalize quaternion
		f_transform.f_quat = glm::normalize(f_transform.f_quat);
		// Set midpoint angular momentum
		f_angularMomentum += delta * f_torque / 2.f;
		// Set midpoint inertia tensor
		calculateInertiaTensorInv();
		// Set midpoint angular velocity
		calculateAngularVelocity();

		// MP integrate
		f_transform.f_quat = initialQuat + (delta / 2.f) 
			* glm::quat(0.f, f_angularVelocity[0], f_angularVelocity[1], f_angularVelocity[2])
			* f_transform.f_quat;
		// Normalize quaternion
		f_transform.f_quat = glm::normalize(f_transform.f_quat);
		// Integrate angular momentum
		f_angularMomentum += delta * f_torque / 2.f;
		// Update inertia tensor
		calculateInertiaTensorInv();
		// Calculate angular velocity
		calculateAngularVelocity();

	}

	friend std::ostream& operator<<(std::ostream& os, const RigidObject3D& object) {
		os << std::setprecision(3)
		<< "    <^> Mass:   "
		<< object.f_mass << std::endl
		<< "    <^> Transform:   " << std::endl
		<< object.f_transform << std::endl
		<< "    <^> Velocity:   ("
		<< object.f_velocity[0] << "; "
		<< object.f_velocity[1] << "; "
		<< object.f_velocity[2] << ")" << std::endl
		<< "    <^> Angular velocity:   ("
		<< object.f_angularVelocity[0] << "; "
		<< object.f_angularVelocity[1] << "; "
		<< object.f_angularVelocity[2] << ")" << std::endl
		<< "    <^> Central force:   ("
		<< object.f_centralForce[0] << "; "
		<< object.f_centralForce[1] << "; "
		<< object.f_centralForce[2] << ")" << std::endl
		<< "    <^> Torque:   ("
		<< object.f_torque[0] << "; "
		<< object.f_torque[1] << "; "
		<< object.f_torque[2] << ")";

		return os;
	}

	virtual void onDraw(Renderer &renderer) {};
};

struct Box: public RigidObject3D {

	#pragma region Constructors
	Box(): RigidObject3D()
	{}

	Box(
		float mass,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::quat& quat,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	RigidObject3D(
		mass,
		position,
		scale,
		quat,
		velocity,
		angularVelocity
	)
	{
		calculateInertiaTensorInv();
		calculateAngularMomentum();
	}

	Box(
		float mass,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::vec3& angles,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	RigidObject3D(
		mass,
		position,
		scale,
		angles,
		velocity,
		angularVelocity
	)
	{
		calculateInertiaTensorInv();
		calculateAngularMomentum();
	}

	Box(
		float mass,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::quat&& quat,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	RigidObject3D(
		mass,
		std::move(position),
		std::move(scale),
		std::move(quat),
		std::move(velocity),
		std::move(angularVelocity)
	)
	{
		calculateInertiaTensorInv();
		calculateAngularMomentum();
	}

	Box(
		float mass,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::vec3&& angles,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	RigidObject3D(
		mass,
		std::move(position),
		std::move(scale),
		std::move(angles),
		std::move(velocity),
		std::move(angularVelocity)
	)
	{
		calculateInertiaTensorInv();
		calculateAngularMomentum();
	}
	#pragma endregion Constructors
	
	glm::mat3 getInitialInertiaTensorInv() const override {
		float Ixx = 12 / (f_mass * (f_transform.f_scale[1] * f_transform.f_scale[1] + f_transform.f_scale[2] * f_transform.f_scale[2]));
		float Iyy = 12 / (f_mass * (f_transform.f_scale[0] * f_transform.f_scale[0] + f_transform.f_scale[2] * f_transform.f_scale[2]));
		float Izz = 12 / (f_mass * (f_transform.f_scale[0] * f_transform.f_scale[0] + f_transform.f_scale[1] * f_transform.f_scale[1]));

		glm::mat3 inertiaTensorInv(
			Ixx, 0.f, 0.f,
			0.f, Iyy, 0.f,
			0.f, 0.f, Izz
		);

		return inertiaTensorInv;
	}

	void onDraw(Renderer &renderer) {
		renderer.drawCube(
			f_transform.f_position,
			f_transform.f_quat,
			f_transform.f_scale,
			glm::vec4(0.7f, 0.7f, 0.7f, 1.f)
		);
		renderer.drawLine(
			f_transform.f_position,
			f_transform.f_position + 0.5f * f_angularVelocity,
			glm::vec3(1.f, 0.05f, 0.05f)
		);
		renderer.drawSphere(
			f_transform.f_position + 0.5f * f_angularVelocity,
			0.03f,
			glm::vec4(1.f, 0.05f, 0.05f, 1.f)
		);
	}
};
