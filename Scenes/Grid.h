#pragma once
#include "Renderer.h"

namespace Grid {

    class Grid2D {
    private:
        // Grid dimensions
        // Dimension wrt X
        unsigned f_n; 
        // Dimension wrt Y
        unsigned f_m;
        
        // Grid bottom left corner coordinates
        glm::dvec2 f_origin;

        // Grid size
        glm::dvec2 f_size;

        // Grid step
        glm::dvec2 f_steps;
    
    public:
    
        #pragma region Constructors

        Grid2D():
        f_n(0),
        f_m(0),
        f_origin(0.),
        f_size(0.),
        f_steps(0.)
        {}

        Grid2D(
            unsigned n,
            unsigned m,
            const glm::dvec2& origin,
            const glm::dvec2& size
        ):
        f_n(n),
        f_m(m),
        f_origin(origin),
        f_size(size) 
        {
            calculateSteps();
        }

        Grid2D(
            unsigned n,
            unsigned m,
            glm::dvec2&& origin,
            glm::dvec2&& size
        ):
        f_n(n),
        f_m(m),
        f_origin(std::move(origin)),
        f_size(std::move(size)) 
        {
            calculateSteps();
        }

        #pragma endregion

        #pragma region Getters and Setters

        unsigned getN() const {return f_n;}

        unsigned getM() const {return f_m;}

        glm::dvec2 getOrigin() const {return f_origin;}

        glm::dvec2 getSize() const {return f_size;}

        glm::dvec2 getSteps() const {return f_steps;}

        double getStepX() const {return f_steps.x;}

        double getStepY() const {return f_steps.y;}

        void setN(unsigned n) {
            f_n = n;
            calculateSteps();
        }

        void setM(unsigned m) {
            f_m = m;
            calculateSteps();
        }

        void setNM(unsigned n, unsigned m) {
            f_n = n;
            f_m = m;
            calculateSteps();
        }

        void setOrigin(const glm::dvec2& origin) {f_origin = origin;}

        void setSize(const glm::dvec2& size) {
            f_size = size;
            calculateSteps();
        }

        void clear() {
            f_n = 0;
            f_m = 0;

            f_origin = glm::dvec2(0.);
            f_size = glm::dvec2(0.);
            f_steps = glm::dvec2(0.);
        }

        #pragma endregion

        void calculateSteps() {
            // Calculate steps according to construtor data
            f_steps.x = f_size.x / (double)(f_n - 1);
            f_steps.y = f_size.y / (double)(f_m - 1);
        }

        glm::dvec2 getPoint(unsigned i, unsigned j) const {
            // Check if invalid indices
            if (i >= f_n || j >= f_m) return f_origin;

            // Otherwise calculate point coordinates
            return glm::dvec2(
                f_origin.x + f_steps.x * i,
                f_origin.y + f_steps.y * j
            );
        }

        glm::dvec3 getPoint3D(unsigned i, unsigned j) const {
            // Check if invalid indices
            if (i >= f_n || j >= f_m) return glm::dvec3(f_origin.x, f_origin.y, -2.5);

            // Otherwise calculate point coordinates
            return glm::dvec3(
                f_origin.x + f_steps.x * i,
                f_origin.y + f_steps.y * j,
                -2.5
            );
        }

        void onDraw(Renderer &renderer) {
            for(unsigned i = 0; i < f_n; i++) {
                renderer.drawLine(getPoint3D(i, 0), getPoint3D(i, f_m - 1), glm::vec3(0.5, 0.5, 0.5));
            }
            for(unsigned j = 0; j < f_m; j++) {
                renderer.drawLine(getPoint3D(0, j), getPoint3D(f_n - 1, j), glm::vec3(0.5, 0.5, 0.5));
            }
        }
    };

}
