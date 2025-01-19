#pragma once

#include "Renderer.h"
#include <memory>

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