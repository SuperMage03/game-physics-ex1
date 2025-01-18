#pragma once

#include "Renderer.h"

/** @brief
 * Force structure.
 * Encompases force vector and application position.
 */
struct Force {
	/// @brief Force vector
	glm::dvec3 f_force;
	/// @brief Force application point
	glm::dvec3 f_applicationPoint;

    #pragma region Constructors

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

    #pragma endregion

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