#pragma once
#include "Renderer.h"

struct Wall
{
    glm::dvec3 f_normal;
    double f_surface;

    Wall():
    f_normal(glm::dvec3(0.f)),
    f_surface(0.f)
    {}

    Wall(
        const glm::dvec3& normal,
        double surface
    ):
    f_normal(normal),
    f_surface(surface)
    {}

    bool pointInside(const glm::dvec3& point) const {
        return glm::dot(point, f_normal) <= f_surface;
    }

    unsigned pointInside(const std::vector<glm::dvec3>& points) const {
        unsigned maxDistInsideId = -1;
        unsigned id = 0;
        double maxDistInside = 0.f;
        for (const auto point: points) {
            double distInside = f_surface - glm::dot(point, f_normal);
            if (maxDistInside <= distInside) {
                maxDistInside = distInside;
                maxDistInsideId = id;
            }
            id++;
        }
        return maxDistInsideId;
    }
};
