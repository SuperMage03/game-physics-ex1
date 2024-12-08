#pragma once
#include "Renderer.h"
#include <iostream>
#include <glm/gtx/quaternion.hpp>

struct Force {
	// Force vector
	glm::dvec3 f_force;
	// Force application point
	glm::dvec3 f_applicationPoint;

	Force():
	f_force(0.),
	f_applicationPoint(0.)
	{}

	Force(
		const glm::dvec3& force,
		const glm::dvec3& applicationPoint
	):
	f_force(force),
	f_applicationPoint(applicationPoint)
	{}

	Force(
		glm::dvec3&& force,
		glm::dvec3&& applicationPoint
	):
	f_force(std::move(force)),
	f_applicationPoint(std::move(applicationPoint))
	{}


	void onDraw(Renderer &renderer) {
		renderer.drawLine(
			f_applicationPoint,
			f_applicationPoint + f_force,
			glm::dvec3(0.15, 0.15, 1.)
		);
		renderer.drawSphere(
			f_applicationPoint + f_force,
			0.01,
			glm::dvec4(0.15, 0.15, 1., 1.)
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
	glm::dvec3 f_position;
	// Object scale, [s_x, s_y, s_z]
	glm::dvec3 f_scale;
	// Object rotation, quaternion
	glm::dquat f_quat;

	Transform3D():
	f_position(0.),
	f_scale(1.),
	f_quat(glm::dvec3(0.))
	{}

	Transform3D(
		const glm::dvec3& position,
		const glm::dvec3& scale,
		const glm::dquat& quat
	):
	f_position(position),
	f_scale(scale),
	f_quat(quat)
	{}

	Transform3D(
		const glm::dvec3& position,
		const glm::dvec3& scale,
		const glm::dvec3& angles
	):
	f_position(position),
	f_scale(scale),
	f_quat(angles)
	{}

	glm::dmat3 getRotationMatrix() const {
		return glm::toMat3(f_quat);
	}

	glm::dmat4 getTransform() const {
		glm::dmat4 rotationMatrix = glm::toMat4(f_quat);
		glm::dmat4 scaleMatrix = glm::scale(glm::dmat4(1.), f_scale);
		glm::dmat4 translationMatrix = glm::translate(glm::dmat4(1.), f_position);
		glm::dmat4 transform = translationMatrix * rotationMatrix * scaleMatrix;
		return transform;
	}

	glm::dvec3 transformLocalToGlobal(const glm::dvec3 position) const {
		return getRotationMatrix() * position + f_position;
	}

	glm::dvec3 transformGlobalToLocal(const glm::dvec3 position) const {
		return glm::inverse(getRotationMatrix()) * position - glm::inverse(getRotationMatrix()) * f_position;
	}

	glm::dvec3 transformLocalToGlobalRotation(const glm::dvec3 vector) const {
		return getRotationMatrix() * vector;
	}

	glm::dvec3 transformGlobalToLocalRotation(const glm::dvec3 vector) const {
		return glm::inverse(getRotationMatrix()) * vector;
	}

	Force transformLocalForceToGlobalForce(const Force& force) const {
		glm::dvec3 globalApplicationPoint = transformLocalToGlobal(force.f_applicationPoint);
		glm::dvec3 globalForce = getRotationMatrix() * force.f_force;
		return Force(globalForce, globalApplicationPoint);
	}

	Force transformGlobalForceToLocalForce(const Force& force) const {
		glm::dvec3 localApplicationPoint = transformGlobalToLocal(force.f_applicationPoint);
		glm::dvec3 localForce = glm::inverse(getRotationMatrix()) * force.f_force;
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
		<< transform.f_quat.w << "; "
		<< transform.f_quat.x << "; "
		<< transform.f_quat.y << "; "
		<< transform.f_quat.z << ")";

		return os;
	}
};

struct RigidObject3D {
	// Object mass
	double f_mass;
	// Current Inertia tensor
	glm::dmat3 f_inertiaTensorInv;
	// Elasticity/Plasticity
	double f_c;
	// Coefficient of friction
	double f_mu;

	// Object transform
	Transform3D f_transform;
	// Object velocity
	glm::dvec3 f_velocity;
	// Angular momentum
	glm::dvec3 f_angularMomentum;
	// Angular velocity
	glm::dvec3 f_angularVelocity;

	// Central force applied to the object
	glm::dvec3 f_centralForce;
	// Torque applied to the object
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
		const glm::dvec3& position,
		const glm::dvec3& scale,
		const glm::dquat& quat,
		const glm::dvec3& velocity,
		const glm::dvec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.),
	f_c(c),
	f_mu(mu),

	f_transform(position, scale, quat),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.),
	f_torque(0.)
	{}

	RigidObject3D(
		double mass,
		double c,
		double mu,
		const glm::dvec3& position,
		const glm::dvec3& scale,
		const glm::dvec3& angles,
		const glm::dvec3& velocity,
		const glm::dvec3& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.),
	f_c(c),
	f_mu(mu),

	f_transform(position, scale, angles),
	f_velocity(velocity),
	f_angularVelocity(angularVelocity),

	f_centralForce(0.),
	f_torque(0.)
	{}

	RigidObject3D(
		double mass,
		double c,
		double mu,
		glm::dvec3&& position,
		glm::dvec3&& scale,
		glm::dquat&& quat,
		glm::dvec3&& velocity,
		glm::dvec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.),
	f_c(c),
	f_mu(mu),

	f_transform(std::move(position), std::move(scale), std::move(quat)),
	f_velocity(std::move(velocity)),
	f_angularVelocity(std::move(angularVelocity)),

	f_centralForce(0.),
	f_torque(0.)
	{}

	RigidObject3D(
		double mass,
		double c,
		double mu,
		glm::dvec3&& position,
		glm::dvec3&& scale,
		glm::dvec3&& angles,
		glm::dvec3&& velocity,
		glm::dvec3&& angularVelocity
	):
	f_mass(mass),
	f_inertiaTensorInv(0.),
	f_c(c),
	f_mu(mu),

	f_transform(std::move(position), std::move(scale), std::move(angles)),
	f_velocity(std::move(velocity)),
	f_angularVelocity(std::move(angularVelocity)),

	f_centralForce(0.),
	f_torque(0.)
	{}
	#pragma endregion Constructors

	void flushForces() {
		f_centralForce = glm::dvec3(0.);
		f_torque = glm::dvec3(0.);
	}

	virtual void calculateInertiaTensorInv() {
		glm::dmat3 rotationMatrix = f_transform.getRotationMatrix();
		glm::dmat3 iiti = getInitialInertiaTensorInv();
		f_inertiaTensorInv = rotationMatrix * iiti * glm::transpose(rotationMatrix);
	}

	virtual glm::dmat3 getInitialInertiaTensorInv() const {
		return glm::identity<glm::dmat3>();
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

	void applyImpulse(const glm::dvec3& impulse, const glm::dvec3& applicationPoint) {
		glm::dvec3 centralDirection = glm::normalize(applicationPoint - f_transform.f_position);
		glm::dvec3 centralImpulse = glm::dot(centralDirection, impulse) * centralDirection;
		glm::dvec3 rotationalImpulse = impulse - centralImpulse;

		f_velocity += centralImpulse / f_mass;
		f_angularMomentum += f_inertiaTensorInv * rotationalImpulse;
		calculateAngularVelocity();
	}

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

struct Box: public RigidObject3D {
	glm::vec4 f_color = glm::vec4(0.7f, 0.7f, 0.7f, 1.f);

	#pragma region Constructors
	Box(): RigidObject3D()
	{}

	Box(
		double mass,
		double c,
		double mu,
		const glm::dvec3& position,
		const glm::dvec3& scale,
		const glm::dquat& quat,
		const glm::dvec3& velocity,
		const glm::dvec3& angularVelocity
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
		double mass,
		double c,
		double mu,
		const glm::dvec3& position,
		const glm::dvec3& scale,
		const glm::dvec3& angles,
		const glm::dvec3& velocity,
		const glm::dvec3& angularVelocity
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
		double mass,
		double c,
		double mu,
		glm::dvec3&& position,
		glm::dvec3&& scale,
		glm::dquat&& quat,
		glm::dvec3&& velocity,
		glm::dvec3&& angularVelocity
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
		double mass,
		double c,
		double mu,
		glm::dvec3&& position,
		glm::dvec3&& scale,
		glm::dvec3&& angles,
		glm::dvec3&& velocity,
		glm::dvec3&& angularVelocity
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
	
	glm::dmat3 getInitialInertiaTensorInv() const override {
		double Ixx = 12. / (f_mass * (f_transform.f_scale[1] * f_transform.f_scale[1] + f_transform.f_scale[2] * f_transform.f_scale[2]));
		double Iyy = 12. / (f_mass * (f_transform.f_scale[0] * f_transform.f_scale[0] + f_transform.f_scale[2] * f_transform.f_scale[2]));
		double Izz = 12. / (f_mass * (f_transform.f_scale[0] * f_transform.f_scale[0] + f_transform.f_scale[1] * f_transform.f_scale[1]));

		glm::dmat3 inertiaTensorInv(
			Ixx, 0., 0.,
			0., Iyy, 0.,
			0., 0., Izz
		);

		return inertiaTensorInv;
	}

	bool containsPoint(const glm::dvec3& point) const {
		glm::dvec3 pointLocal = f_transform.transformGlobalToLocal(point);
		return (abs(pointLocal[0]) <= f_transform.f_scale[0] / 2.)
			&& (abs(pointLocal[1]) <= f_transform.f_scale[1] / 2.)
			&& (abs(pointLocal[2]) <= f_transform.f_scale[2] / 2.);
	}

	unsigned containsPoint(const std::vector<glm::dvec3> points) const {
		unsigned maxDistInsideId = -1;
        unsigned id = 0;
        double maxDistInside = 0.;
		for (const auto point: points) {
			if (containsPoint(point)) return id;
			id++;
		}
		return -1;
	}

	glm::dvec3 getNormalForCollisionPoint(const glm::dvec3 collsionPoint) const {
		// Transform to local coordinates
		glm::dvec3 collsionPointLocal = f_transform.transformGlobalToLocal(collsionPoint);
		// Create distances to cuboid surfaces array
		double distancesToSurfaces[6];
		// -X, X, -Y, Y, -Z, Z
		distancesToSurfaces[0] = abs(collsionPointLocal[0] - (-f_transform.f_scale[0] / 2.));
		distancesToSurfaces[1] = abs(collsionPointLocal[0] - (f_transform.f_scale[0] / 2.));
		distancesToSurfaces[2] = abs(collsionPointLocal[1] - (-f_transform.f_scale[1] / 2.));
		distancesToSurfaces[3] = abs(collsionPointLocal[1] - (f_transform.f_scale[1] / 2.));
		distancesToSurfaces[4] = abs(collsionPointLocal[2] - (-f_transform.f_scale[2] / 2.));
		distancesToSurfaces[5] = abs(collsionPointLocal[2] - (f_transform.f_scale[2] / 2.));

		// Find surface with min distance to collision point
		unsigned minDistanceSurfaceId = 0;
		double minDistance = distancesToSurfaces[0];

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
			return glm::dvec3(-1., 0., 0.);
			break;
		case 1:
			return glm::dvec3(1., 0., 0.);
			break;
		case 2:
			return glm::dvec3(0., -1., 0.);
			break;
		case 3:
			return glm::dvec3(0., 1., 0.);
			break;
		case 4:
			return glm::dvec3(0., 0., -1.);
			break;
		case 5:
			return glm::dvec3(0., 0., 1.);
			break;
		default:
			return glm::dvec3(0.);
			break;
		}
	}

	std::vector<glm::dvec3> getCornerPoints() const {
		std::vector<glm::dvec3> cornerPoints;
		for (int id = 0; id < 8; id++) {
			cornerPoints.emplace_back(std::move(getCornerPointId(id)));
		}
		return cornerPoints;
	}

	glm::dvec3 getCornerPointLocalId(unsigned id) const {
		switch (id)
		{
		case 0:
			return std::move(glm::dvec3(
					-f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 1:
			return std::move(glm::dvec3(
					-f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		case 2:
			return std::move(glm::dvec3(
					-f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 3:
			return std::move(glm::dvec3(
					-f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		case 4:
			return std::move(glm::dvec3(
					f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 5:
			return std::move(glm::dvec3(
					f_transform.f_scale[0] / 2., 
					-f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		case 6:
			return std::move(glm::dvec3(
					f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					-f_transform.f_scale[2] / 2.
			));
			break;
		case 7:
			return std::move(glm::dvec3(
					f_transform.f_scale[0] / 2., 
					f_transform.f_scale[1] / 2., 
					f_transform.f_scale[2] / 2.
			));
			break;
		default:
			return glm::dvec3(0.);
			break;
		}
	}

	glm::dvec3 getCornerPointId(unsigned id) const {
		return std::move(f_transform.transformLocalToGlobal(getCornerPointLocalId(id)));
	}

	glm::dvec3 getCornerPointVelocityId(unsigned id) const {
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
			f_transform.f_position + 0.5 * f_angularVelocity,
			glm::dvec3(1., 0.05, 0.05)
		);
		renderer.drawSphere(
			f_transform.f_position + 0.5 * f_angularVelocity,
			0.03,
			glm::dvec4(1., 0.05, 0.05, 1.)
		);
	}

	
};

struct Collision {
 	std::shared_ptr<RigidObject3D> f_objA;
    std::shared_ptr<RigidObject3D> f_objB;
	glm::dvec3 f_collisionPoint;
	glm::dvec3 f_collisionPointALocal;
	glm::dvec3 f_collisionPointBLocal;
	glm::dvec3 f_relativeVelocity;
	glm::dvec3 f_normal;
	glm::dvec3 f_tangent;

	double f_impulse;

    Collision():
    f_objA(nullptr),
    f_objB(nullptr),
	f_collisionPoint(0.),
	f_collisionPointALocal(0.),
	f_collisionPointBLocal(0.),
	f_relativeVelocity(0.),
	f_normal(0.),
	f_tangent(0.),
	f_impulse(0.)
    {}

	Collision(
		std::shared_ptr<RigidObject3D> objA, 
		std::shared_ptr<RigidObject3D> objB,
		glm::dvec3 collisionPoint,
		glm::dvec3 normal
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

    Collision(
		RigidObject3D* objA, 
		RigidObject3D* objB,
		glm::dvec3 collisionPoint,
		glm::dvec3 normal
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
		f_collisionPointALocal = f_collisionPoint - f_objA->f_transform.f_position;
		f_collisionPointBLocal = f_collisionPoint - f_objB->f_transform.f_position;

		// Calculate collision point velocities
		glm::dvec3 collPointVelocityA = f_objA->f_velocity + glm::cross(f_objA->f_angularVelocity, f_collisionPointALocal);
		glm::dvec3 collPointVelocityB = f_objB->f_velocity + glm::cross(f_objB->f_angularVelocity, f_collisionPointBLocal);

		// Calculate relative velocity
		f_relativeVelocity = collPointVelocityA - collPointVelocityB;
	}

	void calculateTangent() {
		glm::dvec3 tangent = glm::cross(glm::cross(f_normal, f_relativeVelocity), f_normal);
		f_tangent = glm::dvec3(0.);
		if (glm::length(tangent) != 0) f_tangent = glm::normalize(tangent);
	}

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