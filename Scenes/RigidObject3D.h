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
		glm::mat4 rotationMatrix = glm::toMat4(f_quat);
		glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.f), f_scale);
		glm::mat4 translationMatrix = glm::translate(glm::mat4(1.f), f_position);
		glm::mat4 transform = translationMatrix * rotationMatrix * scaleMatrix;
		return transform;
	}

	glm::vec3 transformLocalToGlobal(const glm::vec3 position) const {
		return getRotationMatrix() * position + f_position;
	}

	glm::vec3 transformGlobalToLocal(const glm::vec3 position) const {
		return glm::inverse(getRotationMatrix()) * position - glm::inverse(getRotationMatrix()) * f_position;
	}

	glm::vec3 transformLocalToGlobalRotation(const glm::vec3 vector) const {
		return getRotationMatrix() * vector;
	}

	glm::vec3 transformGlobalToLocalRotation(const glm::vec3 vector) const {
		return glm::inverse(getRotationMatrix()) * vector;
	}

	Force transformLocalForceToGlobalForce(const Force& force) const {
		glm::vec3 globalApplicationPoint = transformLocalToGlobal(force.f_applicationPoint);
		glm::vec3 globalForce = getRotationMatrix() * force.f_force;
		return Force(globalForce, globalApplicationPoint);
	}

	Force transformGlobalForceToLocalForce(const Force& force) const {
		glm::vec3 localApplicationPoint = transformGlobalToLocal(force.f_applicationPoint);
		glm::vec3 localForce = glm::inverse(getRotationMatrix()) * force.f_force;
		return Force(localForce, localApplicationPoint);
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
};

struct RigidObject3D {
	// Object mass
	float f_mass;
	// Current Inertia tensor
	glm::mat3 f_inertiaTensorInv;
	// Elasticity/Plasticity
	float f_c;
	// Coefficient of friction
	float f_mu;

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
	f_c(0.f),
	f_mu(0.f),

	f_transform(),
	f_velocity(0.f),
	f_angularMomentum(0.f),
	f_angularVelocity(0.f),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		float c,
		float mu,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::quat& quat,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),
	f_c(c),
	f_mu(mu),

	f_transform(position, scale, quat),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		float c,
		float mu,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::vec3& angles,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),
	f_c(c),
	f_mu(mu),

	f_transform(position, scale, angles),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		float c,
		float mu,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::quat&& quat,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),
	f_c(c),
	f_mu(mu),

	f_transform(std::move(position), std::move(scale), std::move(quat)),
	f_velocity(std::move(velocity)),
	f_angularVelocity(std::move(angularVelocity)),

	f_centralForce(0.f),
	f_torque(0.f)
	{}

	RigidObject3D(
		float mass,
		float c,
		float mu,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::vec3&& angles,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.f),
	f_c(c),
	f_mu(mu),

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
		// Integrate position
		f_transform.f_position += delta * f_velocity;
		// Integrate linear velocity
		f_velocity += delta * f_centralForce / f_mass;

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
		// Leapfrog for linear movement
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
		<< "        <^> Angular velocity:   ("
		<< object.f_angularVelocity[0] << "; "
		<< object.f_angularVelocity[1] << "; "
		<< object.f_angularVelocity[2] << ")";

		return os;
	}

	virtual void onDraw(Renderer &renderer) {};
};

struct Box: public RigidObject3D {
	glm::vec4 f_color = glm::vec4(0.7f, 0.7f, 0.7f, 1.f);

	#pragma region Constructors
	Box(): RigidObject3D()
	{}

	Box(
		float mass,
		float c,
		float mu,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::quat& quat,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	RigidObject3D(
		mass,
		c,
		mu,
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
		float c,
		float mu,
		const glm::vec3& position,
		const glm::vec3& scale,
		const glm::vec3& angles,
		const glm::vec3& velocity,
		const glm::vec3& angularVelocity
	):
	RigidObject3D(
		mass,
		c,
		mu,
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
		float c,
		float mu,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::quat&& quat,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	RigidObject3D(
		mass,
		c,
		mu,
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
		float c,
		float mu,
		glm::vec3&& position,
		glm::vec3&& scale,
		glm::vec3&& angles,
		glm::vec3&& velocity,
		glm::vec3&& angularVelocity
	):
	RigidObject3D(
		mass,
		c,
		mu,
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

	bool containsPoint(const glm::vec3& point) const {
		glm::vec3 pointLocal = f_transform.transformGlobalToLocal(point);
		return (abs(pointLocal[0]) <= f_transform.f_scale[0] / 2.f)
			&& (abs(pointLocal[1]) <= f_transform.f_scale[1] / 2.f)
			&& (abs(pointLocal[2]) <= f_transform.f_scale[2] / 2.f);
	}

	unsigned containsPoint(const std::vector<glm::vec3> points) const {
		unsigned maxDistInsideId = -1;
        unsigned id = 0;
        float maxDistInside = 0.f;
		for (const auto point: points) {
			if (containsPoint(point)) return id;
			id++;
		}
		return -1;
	}

	glm::vec3 getNormalForCollisionPoint(const glm::vec3 collsionPoint) const {
		// Transform to local coordinates
		glm::vec3 collsionPointLocal = f_transform.transformGlobalToLocal(collsionPoint);
		// Create distances to cuboid surfaces array
		float distancesToSurfaces[6];
		// -X, X, -Y, Y, -Z, Z
		distancesToSurfaces[0] = abs(collsionPointLocal[0] - (-f_transform.f_scale[0] / 2.f));
		distancesToSurfaces[1] = abs(collsionPointLocal[0] - (f_transform.f_scale[0] / 2.f));
		distancesToSurfaces[2] = abs(collsionPointLocal[1] - (-f_transform.f_scale[1] / 2.f));
		distancesToSurfaces[3] = abs(collsionPointLocal[1] - (f_transform.f_scale[1] / 2.f));
		distancesToSurfaces[4] = abs(collsionPointLocal[2] - (-f_transform.f_scale[2] / 2.f));
		distancesToSurfaces[5] = abs(collsionPointLocal[2] - (f_transform.f_scale[2] / 2.f));

		// Find surface with min distance to collision point
		unsigned minDistanceSurfaceId = 0;
		float minDistance = distancesToSurfaces[0];

		for (int i = 1; i < 6; i++) {
			if (minDistance > distancesToSurfaces[i]) {
				minDistance = distancesToSurfaces[i];
				minDistanceSurfaceId = i;
			}
		}

		// Return respective normal
		switch (minDistanceSurfaceId)
		{
		case 0:
			return glm::vec3(-1.f, 0.f, 0.f);
			break;
		case 1:
			return glm::vec3(1.f, 0.f, 0.f);
			break;
		case 2:
			return glm::vec3(0.f, -1.f, 0.f);
			break;
		case 3:
			return glm::vec3(0.f, 1.f, 0.f);
			break;
		case 4:
			return glm::vec3(0.f, 0.f, -1.f);
			break;
		case 5:
			return glm::vec3(0.f, 0.f, 1.f);
			break;
		default:
			return glm::vec3(0.f);
			break;
		}
	}

	std::vector<glm::vec3> getCornerPoints() const {
		std::vector<glm::vec3> cornerPoints;
		for (int id = 0; id < 8; id++) {
			cornerPoints.emplace_back(std::move(getCornerPointId(id)));
		}
		return cornerPoints;
	}

	glm::vec3 getCornerPointLocalId(unsigned id) const {
		switch (id)
		{
		case 0:
			return std::move(glm::vec3(
					-f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 1:
			return std::move(glm::vec3(
					-f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		case 2:
			return std::move(glm::vec3(
					-f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 3:
			return std::move(glm::vec3(
					-f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		case 4:
			return std::move(glm::vec3(
					f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 5:
			return std::move(glm::vec3(
					f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		case 6:
			return std::move(glm::vec3(
					f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 7:
			return std::move(glm::vec3(
					f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		default:
			return glm::vec3(0.f);
			break;
		}
	}

	glm::vec3 getCornerPointId(unsigned id) const {
		return std::move(f_transform.transformLocalToGlobal(getCornerPointLocalId(id)));
	}

	glm::vec3 getCornerPointVelocityId(unsigned id) const {
		return std::move(f_velocity + glm::cross(f_angularVelocity, getCornerPointId(id) - f_transform.f_position));
	}

	void onDraw(Renderer &renderer) {
		renderer.drawCube(
			f_transform.f_position,
			f_transform.f_quat,
			f_transform.f_scale,
			f_color
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

struct Collision {
    RigidObject3D* f_objA;
    RigidObject3D* f_objB;
	glm::vec3 f_collisionPoint;
	glm::vec3 f_collisionPointALocal;
	glm::vec3 f_collisionPointBLocal;
	glm::vec3 f_relativeVelocity;
	glm::vec3 f_normal;
	glm::vec3 f_tangent;

	float f_impulse;

    Collision():
    f_objA(nullptr),
    f_objB(nullptr),
	f_collisionPoint(0.f),
	f_collisionPointALocal(0.f),
	f_collisionPointBLocal(0.f),
	f_relativeVelocity(0.f),
	f_normal(0.f),
	f_tangent(0.f),
	f_impulse(0.f)
    {}

    Collision(
		RigidObject3D* objA, 
		RigidObject3D* objB,
		glm::vec3 collisionPoint,
		glm::vec3 normal
	):
    f_objA(objA),
    f_objB(objB),
	f_collisionPoint(collisionPoint),
	f_normal(normal)
    {
		calculateRelativeVelocity();
		calculateTangent();
		calculateImpulse();
	}

private:
	void calculateRelativeVelocity() {
		// Calculate local collision point coordinates
		f_collisionPointALocal = f_objA->f_transform.transformLocalToGlobalRotation(f_objA->f_transform.transformGlobalToLocal(f_collisionPoint));
		f_collisionPointBLocal = f_objB->f_transform.transformLocalToGlobalRotation(f_objB->f_transform.transformGlobalToLocal(f_collisionPoint));

		// Calculate collision point velocities
		glm::vec3 collPointVelocityA = f_objA->f_velocity + glm::cross(f_objA->f_angularVelocity, f_collisionPointALocal);
		glm::vec3 collPointVelocityB = f_objB->f_velocity + glm::cross(f_objB->f_angularVelocity, f_collisionPointBLocal);

		// Calculate relative velocity
		f_relativeVelocity = collPointVelocityA - collPointVelocityB;
	}

	void calculateTangent() {
		glm::vec3 tangent = glm::cross(glm::cross(f_normal, f_relativeVelocity), f_normal);
		f_tangent = glm::vec3(0.f);
		if (glm::length(tangent) != 0) f_tangent = glm::normalize(tangent);
	}

	void calculateImpulse() {
		f_impulse =
			-(1 + (f_objA->f_c + f_objB->f_c) / 2.f) * std::min(glm::dot(f_relativeVelocity, f_normal), 0.f)
			/ (
				1.f / f_objA->f_mass
				+ 1.f / f_objB->f_mass
				+ glm::dot(glm::cross((f_objA->f_inertiaTensorInv * glm::cross(f_collisionPointALocal, f_normal)), f_collisionPointALocal), f_normal)
				+ glm::dot(glm::cross((f_objB->f_inertiaTensorInv * glm::cross(f_collisionPointBLocal, f_normal)), f_collisionPointBLocal), f_normal)
			);
	}
};