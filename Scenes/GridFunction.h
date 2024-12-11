#pragma once

#include "Grid.h"

namespace GridFunction {

    class IntegerGridScalarFunction2D {
    private:
        // Grid dimensions
        // Number of rows
        unsigned f_n; 
        // Number of columns
        unsigned f_m;

        // Function values
        double** f_values;
    
    private:

        void deleteValues() {
            // Clear rows
            for (unsigned row = 0; row < f_n; row++) {
                delete f_values[row];
            }
            // Clear row pointers
            delete f_values;
        }

        void allocateValues() {
            if (f_n == 0 || f_m == 0) return;
            // Allocate row pointers
            f_values = new double*[f_n];
            // Allocate rows
            for (unsigned row = 0; row < f_n; row++) {
                f_values[row] = new double[f_m];
            }
        }

    public:

        #pragma region Constructors

        IntegerGridScalarFunction2D():
        f_n(0),
        f_m(0),
        f_values(nullptr)
        {}

        IntegerGridScalarFunction2D(const IntegerGridScalarFunction2D& other):
        f_n(other.f_n),
        f_m(other.f_m)
        {
            for (unsigned row = 0; row < other.f_n; row++) {
                for (unsigned col = 0; col < other.f_m; col++) {
                    f_values[row][col] = other.f_values[row][col];
                }
            }
        }

        IntegerGridScalarFunction2D(
            unsigned n,
            unsigned m
        ):
        f_n(n),
        f_m(m)
        {
            allocateValues();
        }

        IntegerGridScalarFunction2D(
            unsigned n,
            unsigned m,
            double value
        ):
        f_n(n),
        f_m(m)
        {
            allocateValues();
            setAllValues(value);
        }

        ~IntegerGridScalarFunction2D() {
            deleteValues();
        }

        #pragma endregion

        #pragma region Getters and Setters

    public:

        unsigned getN() const {return f_n;}

        unsigned getM() const {return f_m;}

        double getValue(unsigned row, unsigned col) {
            // Check if invalid indices
            if (row >= f_n || col >= f_m) return 0.;

            return f_values[row][col];
        }

        bool setValue(unsigned row, unsigned col, double value) {
            // Check if invalid indices
            if (row >= f_n || col >= f_m) return false;

            f_values[row][col] = value;
        }

        void setAllValues(double value) {
            for (unsigned row = 0; row < f_n; row++) {
                for (unsigned col = 0; col < f_m; col++) {
                    f_values[row][col] = 0;
                }
            }
        }

        void clear() {
            f_n = 0;
            f_m = 0;

            deleteValues();
        }

        void reallocate(unsigned n, unsigned m) {
            deleteValues();
            f_n = n;
            f_m = m;

            allocateValues();
        }

        #pragma endregion
    };

    class ScalarGridFunction2D {
    private:
        // Grid
        Grid::Grid2D f_grid;

        // Inreger grid function
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
        
        glm::dvec2 getPoint(unsigned row, unsigned col) const {
            f_grid.getPoint(row, col);
        }

        unsigned getN() const {return f_function.getN();}

        unsigned getM() const {return f_function.getM();}

        Grid::Grid2D getGrid() const {return f_grid;}

        GridFunction::IntegerGridScalarFunction2D getIntegerGridScalarFunction() const {return f_function;}

        double getValue(unsigned row, unsigned col) {
            return f_function.getValue(row, col);
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

        bool setValue(unsigned row, unsigned col, double value) {
            return f_function.setValue(row, col, value);
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
    };

}