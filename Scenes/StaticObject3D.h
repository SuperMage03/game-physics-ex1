#pragma once

#include "Renderer.h"
#include "Transform.h"

/** @brief
 * 3D static object structure.
 * Encompases properties of a static 3D object.
 */
struct StaticObject3D {
    /// Object transform
    Transform3D f_transform;

    #pragma region Constructors

    StaticObject3D():
    f_transform()
    {}

    StaticObject3D(
        const Transform3D& transform
    ):
    f_transform(transform)
    {}

    StaticObject3D(
        Transform3D&& transform
    ):
    f_transform(std::move(transform))
    {}

    #pragma endregion

    virtual void onDraw(Renderer &renderer) {};
};

struct StaticCuboid : public StaticObject3D {
    /// Object color.
    glm::vec4 f_color;

    #pragma region Constructors

    StaticCuboid(): StaticObject3D() {
        f_color = glm::vec4(1.);
    }

    StaticCuboid(
        const Transform3D& transform,
        const glm::vec4 color
    ) : StaticObject3D(transform) {
        f_color = color;
    }

    StaticCuboid(
        Transform3D&& transform,
        glm::vec4&& color
    ) : StaticObject3D(std::move(transform)) {
        f_color = std::move(color);
    }

    #pragma endregion

    void onDraw(Renderer &renderer, bool wire = false) {
        if (!wire) {
            renderer.drawCube(
                f_transform.f_position,
                f_transform.f_quat,
                f_transform.f_scale,
                f_color
            );
        }
        else {
            // Get corner points in local frame
            std::vector<glm::dvec3> localCorners(8);
            localCorners.at(0) = glm::dvec3(-f_transform.f_scale / 2.);
            localCorners.at(1) = glm::dvec3(f_transform.f_scale.x / 2., -f_transform.f_scale.y / 2., -f_transform.f_scale.z / 2.);
            localCorners.at(2) = glm::dvec3(f_transform.f_scale.x / 2., f_transform.f_scale.y / 2., -f_transform.f_scale.z / 2.);
            localCorners.at(3) = glm::dvec3(-f_transform.f_scale.x / 2., f_transform.f_scale.y / 2., -f_transform.f_scale.z / 2.);

            localCorners.at(4) = glm::dvec3(-f_transform.f_scale.x / 2., -f_transform.f_scale.y / 2., f_transform.f_scale.z / 2.);
            localCorners.at(5) = glm::dvec3(f_transform.f_scale.x / 2., -f_transform.f_scale.y / 2., f_transform.f_scale.z / 2.);
            localCorners.at(6) = glm::dvec3(f_transform.f_scale / 2.);
            localCorners.at(7) = glm::dvec3(-f_transform.f_scale.x / 2., f_transform.f_scale.y / 2., f_transform.f_scale.z / 2.);

            // Rotate all points according to quat
            // and move all corners according to translation
            std::vector<glm::dvec3> globalCorners(8);
            glm::dmat3 rotationMx = f_transform.getRotationMatrix();
            for (unsigned i = 0; i < 8; i++) {
                globalCorners.at(i) = rotationMx * localCorners.at(i) + f_transform.f_position;
            }

            // Draw wire cube
            // List of edges by vertex ids
            uint16_t edges[12][2] = {
                {0, 1},
                {1, 2},
                {2, 3},
                {3, 0},
                {0, 4},
                {1, 5},
                {2, 6},
                {3, 7},
                {4, 5},
                {5, 6},
                {6, 7},
                {7, 4}
            };

            for (unsigned i = 0; i < 12; i++) {
                renderer.drawLine(
                    globalCorners.at(edges[i][0]),
                    globalCorners.at(edges[i][1]),
                    f_color
                );
            }
        }
	}
};
