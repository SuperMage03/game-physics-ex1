#pragma once

#include "Grid.h"

/** @brief
 * Namespace of grid based functions.
 */
namespace GridFunction {
    /** @brief
    * Scalar function on a 2D integer grid.
    */ 
    class IntegerGridScalarFunction2D {
    private:
        // Grid dimensions
        /// @brief Number of vetices along Ox.
        unsigned f_n; 
        /// @brief Number of vetices along Oy.
        unsigned f_m;

        /// @brief Function values.
        std::vector<std::vector<double>> f_values;

    public:

        #pragma region Constructors

        IntegerGridScalarFunction2D():
        f_n(0),
        f_m(0),
        f_values(0.)
        {}

        IntegerGridScalarFunction2D(const IntegerGridScalarFunction2D& other):
        f_n(other.f_n),
        f_m(other.f_m),
        f_values(other.f_values)
        {}

        IntegerGridScalarFunction2D(IntegerGridScalarFunction2D&& other):
        f_n(other.f_n),
        f_m(other.f_m),
        f_values(std::move(other.f_values))
        {}

        IntegerGridScalarFunction2D(
            unsigned n,
            unsigned m
        ):
        f_n(n),
        f_m(m)
        {
            resize();
        }

        IntegerGridScalarFunction2D(
            unsigned n,
            unsigned m,
            double value
        ):
        f_n(n),
        f_m(m)
        {
            f_values.resize(n);
            for (unsigned i = 0; i < n; i++) {
                f_values[i].resize(m);
                for (unsigned j = 0; j < m; j++) {
                    f_values[i][j] = value;
                }
            }
        }

    private:
        
        void resize() {
            f_values.resize(f_n);
            for (unsigned i = 0; i < f_n; i++) {
                f_values[i].resize(f_m);
            }
        }

        #pragma endregion

        #pragma region Getters and Setters

    public:

        unsigned getN() const {return f_n;}

        unsigned getM() const {return f_m;}

        double getValue(unsigned i, unsigned j) {
            // Check if invalid indices
            if (i >= f_n || j >= f_m) return 0.;
            return f_values[i][j];
        }

        bool setValue(unsigned i, unsigned j, double value) {
            // Check if invalid indices
            if (i >= f_n || j >= f_m) return false;

            f_values[i][j] = value;
            return true;
        }

        bool addToValue(unsigned i, unsigned j, double value) {
            if (i >= f_n || j >= f_m) return false;

            f_values[i][j] += value;
            return true;
        }

        void setAllValues(double value) {
            for (unsigned i = 0; i < f_n; i++) {
                for (unsigned j = 0; j < f_m; j++) {
                    f_values[i][j] = value;
                }
            }
        }

        void clear() {
            f_n = 0;
            f_m = 0;

            for (unsigned i = 0; i < f_n; i++) {
                f_values[i].clear();
            }
            f_values.clear();
        }

        void reallocate(unsigned n, unsigned m) {
            clear();
            f_n = n;
            f_m = m;
            resize();
        }

        #pragma endregion

        IntegerGridScalarFunction2D& operator=(const IntegerGridScalarFunction2D& other) {
            if (this == &other) return *this;
            clear();

            f_n = other.f_n;
            f_m = other.f_m;
            f_values = other.f_values;

            return *this;
        }

        IntegerGridScalarFunction2D& operator=(IntegerGridScalarFunction2D&& other) {
            if (this == &other) return *this;
            clear();

            f_n = other.f_n;
            f_m = other.f_m;
            f_values = std::move(other.f_values);

            return *this;
        }
    };

    /** @brief
    * Scalar function on a 2D grid.
    */ 
    class ScalarGridFunction2D {
    private:
        /// @brief 2D grid on which the function is defined.
        Grid::Grid2D f_grid;

        /// @brief Integer grid function that holds values of the function on the respective f_grid points.
        IntegerGridScalarFunction2D f_function;
    
    public:
        #pragma region Constructors

        ScalarGridFunction2D():
        f_grid(),
        f_function()
        {}

        ScalarGridFunction2D(
            unsigned n,
            unsigned m,
            const glm::dvec2& origin,
            const glm::dvec2& size
        ):
        f_grid(n, m, origin, size),
        f_function(n, m)
        {}

        ScalarGridFunction2D(
            unsigned n,
            unsigned m,
            glm::dvec2&& origin,
            glm::dvec2&& size
        ):
        f_grid(n, m, origin, size),
        f_function(n, m)
        {}

        ScalarGridFunction2D(
            unsigned n,
            unsigned m,
            const glm::dvec2& origin,
            const glm::dvec2& size,
            double value
        ):
        f_grid(n, m, origin, size),
        f_function(n, m, value)
        {}

        ScalarGridFunction2D(
            unsigned n,
            unsigned m,
            glm::dvec2&& origin,
            glm::dvec2&& size,
            double value
        ):
        f_grid(n, m, origin, size),
        f_function(n, m, value)
        {}

        #pragma endregion

        #pragma region Getters and Setters
        
        glm::dvec2 getPoint(unsigned i, unsigned j) const {
            return f_grid.getPoint(i, j);
        }

        glm::dvec3 getPoint3D(unsigned i, unsigned j) {
            glm::dvec3 point = f_grid.getPoint3D(i, j);
           point.z = f_function.getValue(i, j);
            return point;
        }

        unsigned getN() const {return f_function.getN();}

        unsigned getM() const {return f_function.getM();}

        Grid::Grid2D getGrid() const {return f_grid;}

        GridFunction::IntegerGridScalarFunction2D getIntegerGridScalarFunction() const {return f_function;}

        double getValue(unsigned i, unsigned j) {
            return f_function.getValue(i, j);
        }

        glm::dvec2 getSteps() const {return f_grid.getSteps();}

        double getStepX() const {return f_grid.getStepX();}
        
        double getStepY() const {return f_grid.getStepY();}

        void setOrigin(const glm::dvec2& origin) {
            f_grid.setOrigin(origin);
        }

        void setSize(const glm::dvec2& size) {
            f_grid.setSize(size);
        }

        bool setValue(unsigned i, unsigned j, double value) {
            return f_function.setValue(i, j, value);
        }

        bool addToValue(unsigned i, unsigned j, double value) {
            return f_function.addToValue(i, j, value);
        }

        void setAllValues(double value) {
            f_function.setAllValues(value);
        }

        void clear() {
            f_function.clear();
            f_grid.clear();
        }

        void reallocate(unsigned n, unsigned m) {
            f_function.reallocate(n, m);
            f_grid.setNM(n, m);
        }

        void reallocate(
            unsigned n, 
            unsigned m,
            const glm::dvec2& origin,
            const glm::dvec2& size
        ) {
            f_function.reallocate(n, m);
            f_grid.setOrigin(origin);
            f_grid.setSize(size);
            f_grid.setNM(n, m);
        }

        #pragma endregion

        /// @brief Determines the color for a specific temperature value.
        /// @param value Z value of a point
        /// @return glm::vec3 color vector of the respective color.
        glm::dvec3 getColorForValue(double value) {
            float base = 3.;
            if (value <= 0.) {
                double t = (value + base) / base;
                return (1. - t) * glm::dvec3(0., 0., 1.) + t * glm::dvec3(1., 0., 0.);
            } 
            else {
                double t = value / base;
                return (1. - t) * glm::dvec3(1., 0., 0.) + t * glm::dvec3(1., 0.8, 0.);
            }
        }

        void onDraw(Renderer &renderer, glm::dvec3 shift = glm::dvec3(0.), double scale = 1.) {
            // Draw grid
            f_grid.onDraw(renderer, shift, scale);
            // Draw function points
            for (unsigned i = 0; i < f_function.getN(); i++) {
                for (unsigned j = 0; j < f_function.getM(); j++) {
                    renderer.drawSphere(
                        shift + scale * getPoint3D(i, j),
                        0.015,
                        glm::vec4(getColorForValue(f_function.getValue(i, j)), 1.)
                    );
                }
            }
            // Draw function bi-linear interpolation lines (except the X1 and Y1 borders)
            for (unsigned i = 0; i < f_function.getN() - 1; i++) {
                for (unsigned j = 0; j < f_function.getM() - 1; j++) {
                    renderer.drawLine(
                        shift + scale * getPoint3D(i, j),
                        shift + scale * getPoint3D(i + 1, j),
                        getColorForValue(f_function.getValue(i, j)),
                        getColorForValue(f_function.getValue(i + 1, j))
                    );
                    renderer.drawLine(
                        shift + scale * getPoint3D(i, j),
                        shift + scale * getPoint3D(i, j + 1),
                        getColorForValue(f_function.getValue(i, j)),
                        getColorForValue(f_function.getValue(i, j + 1))
                    );
                }
            }
            // Draw remaining lines (Y1 border)
            for (unsigned i = 0; i < f_function.getN() - 1; i++) {
                unsigned j = f_function.getM() - 1;
                renderer.drawLine(
                    shift + scale * getPoint3D(i, j),
                    shift + scale * getPoint3D(i + 1, j),
                    getColorForValue(f_function.getValue(i, j)),
                    getColorForValue(f_function.getValue(i + 1, j))
                );
            }
            // Draw remaining lines (X1 border)
            for (unsigned j = 0; j < f_function.getM() - 1; j++) {
                unsigned i = f_function.getN() - 1;
                renderer.drawLine(
                    shift + scale * getPoint3D(i, j),
                    shift + scale * getPoint3D(i, j + 1),
                    getColorForValue(f_function.getValue(i, j)),
                    getColorForValue(f_function.getValue(i, j + 1))
                );
            }
        }
    };
}