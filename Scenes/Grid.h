#pragma once
#include "Renderer.h"

namespace Grid {

    class Grid2D {
    private:
        // Grid dimensions
        // Number of rows
        unsigned f_n; 
        // Number of columns
        unsigned f_m;
        
        // Grid bottom left corner coordinates
        glm::dvec2 f_origin;

        // Grid extent
        glm::dvec2 f_extent;

        // Grid step
        glm::dvec2 f_steps;
    
    public:
        #pragma region Constructors

        Grid2D():
        f_n(0),
        f_m(0),
        f_origin(0.),
        f_extent(0.),
        f_steps(0.)
        {}

        Grid2D(
            unsigned n,
            unsigned m,
            const glm::dvec2& origin,
            const glm::dvec2& extent
        ):
        f_n(n),
        f_m(m),
        f_origin(origin),
        f_extent(extent) 
        {
            calculateSteps();
        }

        Grid2D(
            unsigned n,
            unsigned m,
            glm::dvec2&& origin,
            glm::dvec2&& extent
        ):
        f_n(n),
        f_m(m),
        f_origin(std::move(origin)),
        f_extent(std::move(extent)) 
        {
            calculateSteps();
        }

        #pragma endregion

        #pragma region Getters and Setters

        unsigned getN() {return f_n;}

        unsigned getM() {return f_m;}

        glm::dvec2 getOrigin() {return f_origin;}

        glm::dvec2 getExtent() {return f_extent;}

        glm::dvec2 getSteps() {return f_steps;}

        void setN(unsigned n) {
            f_n = n;
            calculateSteps();
        }

        void setM(unsigned m) {
            f_m = m;
            calculateSteps();
        }

        void setOrigin(const glm::dvec2& origin) {f_origin = origin;}

        void setExtent(const glm::dvec2& extent) {
            f_extent = extent;
            calculateSteps();
        }

        #pragma endregion

        void calculateSteps() {
            // Calculate steps according to construtor data
            f_steps.x = f_extent.x / (double)(f_n - 1);
            f_steps.y = f_extent.y / (double)(f_m - 1);
        }

        glm::dvec2 getPoint(unsigned i, unsigned j) const {
            // Check if invalid indices
            if (i >= f_n || j >= f_m) return f_origin;

            // Otherwise calculate point coordinates
            return glm::dvec2(
                f_origin.x + f_steps.x * i,
                f_origin.y + f_steps.x * i
            );
        }
    };

}
