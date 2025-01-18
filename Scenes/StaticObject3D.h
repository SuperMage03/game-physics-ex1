#pragma once

#include "Renderer.h"
#include "Transform.h"

/** @brief
 * 3D static object structure.
 * Encompases properties of a static 3D object.
 */
struct StaticObject3D {
    /// @brief Object transform
    Transform3D f_transform;

    /// @brief Object color.
    glm::vec4 f_color;

    #pragma region Constructors

    StaticObject3D():
    f_transform(),
    f_color(1.)
    {}

    StaticObject3D(
        const Transform3D& transform,
        const glm::vec4 color
    ):
    f_transform(transform),
    f_color(color)
    {}

    StaticObject3D(
        Transform3D&& transform,
        glm::vec4&& color
    ):
    f_transform(std::move(transform)),
    f_color(color)
    {}

    #pragma endregion

    virtual void onDraw(Renderer &renderer) {};
};


struct StaticCuboid : public StaticObject3D {
    private:

    /// @brief List of vertice coordinate multipliers
    int f_vertices[8][3] = {
      {-1, -1, -1},
      {1, -1, -1},
      {1, 1, -1},
      {-1, 1, -1},
      {-1, -1, 1},
      {1, -1, 1},
      {1, 1, 1},
      {-1, 1, 1},
    };

    /// @brief List of edges by vertex ids
    uint16_t f_edges[12][2] = {
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
    

    #pragma region Constructors

    StaticCuboid(): StaticObject3D() {}

    StaticCuboid(
        const Transform3D& transform,
        const glm::vec4 color
    ) : StaticObject3D(
        transform,
        color
    )
    {}

    StaticCuboid(
        Transform3D&& transform,
        glm::vec4&& color
    ) : StaticObject3D(
        std::move(transform), 
        std::move(color)
    )
    {}

    #pragma endregion

    /// @brief Determines whether the cuboid contains the point.
	/// @param point Point to check.
	/// @return Returns true if the point is inside the cuboid or on the surface, returns false otherwise.
    bool containsPoint(glm::dvec3 point) {
        glm::dvec3 pointLocal = f_transform.transformGlobalToLocal(point);
        return
            pointLocal.x >= -f_transform.f_scale.x / 2. && pointLocal.x <= f_transform.f_scale.x / 2. &&
            pointLocal.y >= -f_transform.f_scale.y / 2. && pointLocal.y <= f_transform.f_scale.y / 2. &&
            pointLocal.z >= -f_transform.f_scale.z / 2. && pointLocal.z <= f_transform.f_scale.z / 2.;
    }

    /// @brief Determines whether a cuboid bigger by margin along x contains the point.
	/// @param point Point to check.
	/// @return Returns true if the point is inside the extended cuboid or on the surface, returns false otherwise.
    bool containsPointExtendedX(glm::dvec3 point, double margin) {
        glm::dvec3 pointLocal = f_transform.transformGlobalToLocal(point);
        return
            pointLocal.x >= -f_transform.f_scale.x / 2. - margin && pointLocal.x <= f_transform.f_scale.x / 2. + margin &&
            pointLocal.y >= -f_transform.f_scale.y / 2. && pointLocal.y <= f_transform.f_scale.y / 2. &&
            pointLocal.z >= -f_transform.f_scale.z / 2. && pointLocal.z <= f_transform.f_scale.z / 2.;
    }

    /// @brief Determines whether a cuboid bigger by margin along x contains the point.
	/// @param point Point to check.
	/// @return Returns true if the point is inside the extended cuboid or on the surface, returns false otherwise.
    bool containsPointExtendedY(glm::dvec3 point, double margin) {
        glm::dvec3 pointLocal = f_transform.transformGlobalToLocal(point);
        return
            pointLocal.x >= -f_transform.f_scale.x / 2. && pointLocal.x <= f_transform.f_scale.x / 2. &&
            pointLocal.y >= -f_transform.f_scale.y / 2. - margin && pointLocal.y <= f_transform.f_scale.y / 2. + margin &&
            pointLocal.z >= -f_transform.f_scale.z / 2. && pointLocal.z <= f_transform.f_scale.z / 2.;
    }

    /// @brief Determines whether a cuboid bigger by margin along x contains the point.
	/// @param point Point to check.
	/// @return Returns true if the point is inside the extended cuboid or on the surface, returns false otherwise.
    bool containsPointExtendedZ(glm::dvec3 point, double margin) {
        glm::dvec3 pointLocal = f_transform.transformGlobalToLocal(point);
        return
            pointLocal.x >= -f_transform.f_scale.x / 2. && pointLocal.x <= f_transform.f_scale.x / 2. &&
            pointLocal.y >= -f_transform.f_scale.y / 2. && pointLocal.y <= f_transform.f_scale.y / 2. &&
            pointLocal.z >= -f_transform.f_scale.z / 2. - margin && pointLocal.z <= f_transform.f_scale.z / 2. + margin;
    }

    /// @brief Calculates coordinates of the corner vertice of a given id
    /// @param id Vertex id.
    /// @return glm::dvec3 corner vertice coordinates
    glm::dvec3 getCornerVertexLocal(int id) {
        glm::dvec3 localVertexCoordinates = glm::dvec3(
            f_vertices[id][0] * f_transform.f_scale.x / 2.,
            f_vertices[id][1] * f_transform.f_scale.y / 2.,
            f_vertices[id][2] * f_transform.f_scale.z / 2.
        );
        return localVertexCoordinates;
    }

    /// @brief Determines whether the cuboid intersects a given ball.
    /// @param position Ball position.
    /// @param radius Ball radius.
    /// @return Returns true if the cuboid intersects the ball (including single point intersections), returns false otherwise.
    bool collidesWithBall(glm::dvec3 position, double radius) {
        // Check if the center of the ball is contained in the cuboids that are extended by radius in every direction
        bool contains = 
               containsPointExtendedX(position, radius) 
            || containsPointExtendedY(position, radius) 
            || containsPointExtendedZ(position, radius);
        
        if (contains) return contains;

        glm::dvec3 positionLocal = f_transform.transformGlobalToLocal(position);
        // Check every vertex separately
        for (unsigned i = 0; i < 8; i++) {
            // Check if ball center is close enough to vertex
            glm::dvec3 vertex = getCornerVertexLocal(i);
            contains = (positionLocal - vertex).length() <= radius;
            if (contains) {
                return contains;
            }
        }

        return false;
    }

    /// @brief Determines whether the cuboid intersects a given ball and returns a normal to the cuboid at intersection.
    /// @param position Ball position.
    /// @param radius Ball radius.
    /// @return Returns the normal at intersection if the cuboid intersects the ball (including single point intersections), returns zero vector otherwise.
    glm::vec3 collidesWithBallNormal(glm::dvec3 position, double radius) {
        glm::dvec3 positionLocal = f_transform.transformGlobalToLocal(position);
        // Check if the center of the ball is contained in the cuboid
        if (containsPoint(position)) {
            // Calculate the minimum distance to X sides
            double distX = std::min(positionLocal.x + f_transform.f_scale.x / 2., f_transform.f_scale.x / 2. - positionLocal.x);
            double distY = std::min(positionLocal.y + f_transform.f_scale.y / 2., f_transform.f_scale.y / 2. - positionLocal.y);
            double distZ = std::min(positionLocal.z + f_transform.f_scale.z / 2., f_transform.f_scale.z / 2. - positionLocal.z);
            
            if (distX <= distY && distX <= distZ) {
                if (positionLocal.x < 0) {
                    return f_transform.transformLocalToGlobalRotation(glm::dvec3(-1., 0., 0.));
                }
                else {
                    return f_transform.transformLocalToGlobalRotation(glm::dvec3(1., 0., 0.));
                }
            }
            else if (distY <= distX && distY <= distZ) {
                if (positionLocal.y < 0) {
                    return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., -1., 0.));
                }
                else {
                    return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., 1., 0.));
                }
            }
            else if (distZ <= distX && distZ <= distY) {
                if (positionLocal.z < 0) {
                    return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., 0., -1.));
                }
                else {
                    return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., 0., 1.));
                }
            }
        }
        // Check if the center of the ball is contained in the cuboids that are extended by radius in every direction
        if (containsPointExtendedX(position, radius)) {
            if (positionLocal.x < 0) {
                return f_transform.transformLocalToGlobalRotation(glm::dvec3(-1., 0., 0.));
            }
            else {
                return f_transform.transformLocalToGlobalRotation(glm::dvec3(1., 0., 0.));
            }
        }
        if (containsPointExtendedY(position, radius)) {
            if (positionLocal.y < 0) {
                return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., -1., 0.));
            }
            else {
                return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., 1., 0.));
            }
        }
        if (containsPointExtendedZ(position, radius)) {
            if (positionLocal.z < 0) {
                return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., 0., -1.));
            }
            else {
                return f_transform.transformLocalToGlobalRotation(glm::dvec3(0., 0., 1.));
            }
        }

        // Check every vertex separately
        for (unsigned i = 0; i < 8; i++) {
            // Check if ball center is close enough to vertex
            glm::dvec3 vertex = getCornerVertexLocal(i);
            double distance = (positionLocal - vertex).length();
            if (distance <= radius) {
                return f_transform.transformLocalToGlobalRotation((positionLocal - vertex) / distance);
            }
        }

        return glm::vec3(0.);
    }


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
            

            for (unsigned i = 0; i < 12; i++) {
                renderer.drawLine(
                    globalCorners.at(f_edges[i][0]),
                    globalCorners.at(f_edges[i][1]),
                    f_color
                );
            }
        }
	}
};
