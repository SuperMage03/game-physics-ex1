#pragma once

#include "Renderer.h"
#include "Force.h"
#include <glm/gtx/quaternion.hpp>

/** @brief
* Transform 3D structure that defines a 3D transformation tensor for a 3D object.
* Encompases position, scale and rotation of a body.
*/ 
struct Transform3D {
	/// Object center of mass position, [x, y, z]
	glm::dvec3 f_position;
	/// Object scale, [s_x, s_y, s_z]
	glm::dvec3 f_scale;
	/// Object rotation, quaternion
	glm::dquat f_quat;

    #pragma region Construtors

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

    Transform3D(
		glm::dvec3&& position,
		glm::dvec3&& scale,
		glm::dquat&& quat
	):
	f_position(std::move(position)),
	f_scale(std::move(scale)),
	f_quat(std::move(quat))
	{}

	Transform3D(
		glm::dvec3&& position,
		glm::dvec3&& scale,
		glm::dvec3&& angles
	):
	f_position(std::move(position)),
	f_scale(std::move(scale)),
	f_quat(std::move(angles))
	{}

    #pragma endregion

	/// @brief Builds rotation matrix from quaternion f_quat.
	/// @return glm::dmat3 rotaion matrix.
	glm::dmat3 getRotationMatrix() const {
		return glm::toMat3(f_quat);
	}

	/// @brief Builds 4x4 transformation matrix from struct fields (Homogeneous coordinates).
	/// @return glm::dmat4 transformation matrix.
	glm::dmat4 getTransform() const {
		glm::dmat4 rotationMatrix = glm::toMat4(f_quat);
		glm::dmat4 scaleMatrix = glm::scale(glm::dmat4(1.), f_scale);
		glm::dmat4 translationMatrix = glm::translate(glm::dmat4(1.), f_position);
		glm::dmat4 transform = translationMatrix * rotationMatrix * scaleMatrix;
		return transform;
	}

	/// @brief Maps a point in the local coordinate frame to a point in the global coordinate frame using current transformation matrix.
	/// @param position Poisition of point in the local coordinate frame.
	/// @return glm::dvec3 position of point in the global coordinate frame.
	glm::dvec3 transformLocalToGlobal(const glm::dvec3 position) const {
		return getRotationMatrix() * position + f_position;
	}

    /// @brief Maps a point in the global coordinate frame to a point in the global coordinate frame using current transformation matrix.
	/// @param position Poisition of point in the global coordinate frame.
	/// @return glm::dvec3 position of point in the local coordinate frame.
	glm::dvec3 transformGlobalToLocal(const glm::dvec3 position) const {
		return glm::inverse(getRotationMatrix()) * position - glm::inverse(getRotationMatrix()) * f_position;
	}

    /// @brief Maps a free vector in the local coordinate frame to a free vector in the global coordinate frame using current transformation matrix.
	/// @param position Free vector in the local coordinate frame.
	/// @return glm::dvec3 free vector in the global coordinate frame.
	glm::dvec3 transformLocalToGlobalRotation(const glm::dvec3 vector) const {
		return getRotationMatrix() * vector;
	}

    /// @brief Maps a free vector in the global coordinate frame to a free vector in the local coordinate frame using current transformation matrix.
	/// @param position Free vector in the global coordinate frame.
	/// @return glm::dvec3 free vector in the local coordinate frame.
	glm::dvec3 transformGlobalToLocalRotation(const glm::dvec3 vector) const {
		return glm::inverse(getRotationMatrix()) * vector;
	}

    /**
     * @brief Maps a Force strucure in the local coordinate frame to a Force strucure in the global coordinate frame using current transformation matrix.
     * Transforms both application point an force vector.
     */ 
	/// @param position Force strucure in the local coordinate frame.
	/// @return Force strucute in the global coordinate frame.
	Force transformLocalForceToGlobalForce(const Force& force) const {
		glm::dvec3 globalApplicationPoint = transformLocalToGlobal(force.f_applicationPoint);
		glm::dvec3 globalForce = getRotationMatrix() * force.f_force;
		return Force(globalForce, globalApplicationPoint);
	}

    /**
     * @brief Maps a Force strucure in the global coordinate frame to a Force strucure in the local coordinate frame using current transformation matrix.
     * Transforms both application point an force vector.
     */ 
	/// @param position Force strucure in the global coordinate frame.
	/// @return Force strucute in the local coordinate frame.
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