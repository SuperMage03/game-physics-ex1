#pragma once

namespace GridFunction {

    class ScalarGridFunction2D {
    private:
        // Grid dimensions
        // Number of rows
        unsigned f_n; 
        // Number of columns
        unsigned f_m;

        // Function values
        double** f_values;
    
    public:

        #pragma region Constructors

        ScalarGridFunction2D():
        f_n(0),
        f_m(0),
        f_values(nullptr)
        {}

        ScalarGridFunction2D(
            unsigned n,
            unsigned m
        ):
        f_n(n),
        f_m(m)
        {
            // Allocated row pointers
            f_values = new double*[n];
            // Allocate rows
            for (int row = 0; row < n; row++) {
                f_values[row] = new double[m];
            }
        }

        ~ScalarGridFunction2D() {
            deleteGrid();
        }

        #pragma endregion

        #pragma region Getters and Setters

        unsigned getN() {return f_n;}

        unsigned getM() {return f_m;}

        bool setValue(unsigned i, unsigned j, double value) {
            // Check if invalid indices
            if (i >= f_n || j >= f_m) return false;

            f_values[i][j] = value;
        }

        void clear() {
            f_n = 0;
            f_m = 0;

            deleteGrid();
        }

        void reallocate(unsigned n, unsigned m) {
            deleteGrid();
            f_n = n;
            f_m = m;

            // Allocated row pointers
            f_values = new double*[n];
            // Allocate rows
            for (int row = 0; row < n; row++) {
                f_values[row] = new double[m];
            }
        }

        #pragma endregion

    private:
        
        void deleteGrid() {
            // Clear rows
            for (int row = 0; row < f_n; row++) {
                delete f_values[row];
            }
            // Clear row pointers
            delete f_values;
        }
    
    public:
        
    };

}