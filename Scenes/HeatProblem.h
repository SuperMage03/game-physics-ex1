#pragma once

#include <functional>
#include "Renderer.h"

namespace HeatProblem {
    typedef std::function<double(glm::dvec2)> ScalarFunction2D;
    typedef std::function<double(double)> ScalarFunction1D;

    struct HeatProblemRectDBC2D {
        // Domain
        // Origin point
        glm::dvec2 f_domainOrigin;
        // Size
        glm::dvec2 f_domainSize;

        // Thermal diffusivity
        double f_mu;
        
        // Initial condition
        ScalarFunction2D f_initialCondition;

        // Boundary conditions
        ScalarFunction1D f_boundaryConditionX0;
        ScalarFunction1D f_boundaryConditionX1;
        ScalarFunction1D f_boundaryConditionY0;
        ScalarFunction1D f_boundaryConditionY1;

        HeatProblemRectDBC2D():
        f_domainOrigin(0.),
        f_domainSize(0.),
        f_mu(0.),
        f_initialCondition(nullptr),
        f_boundaryConditionX0(nullptr),
        f_boundaryConditionX1(nullptr),
        f_boundaryConditionY0(nullptr),
        f_boundaryConditionY1(nullptr)
        {}

        HeatProblemRectDBC2D(
            const glm::dvec2& domainOrigin,
            const glm::dvec2& domainSize,
            double mu,
            const ScalarFunction2D& initialCondition,
            const ScalarFunction1D& boundaryConditionX0,
            const ScalarFunction1D& boundaryConditionX1,
            const ScalarFunction1D& boundaryConditionY0,
            const ScalarFunction1D& boundaryConditionY1
        ):
        f_domainOrigin(domainOrigin),
        f_domainSize(domainSize),
        f_mu(mu),
        f_initialCondition(initialCondition),
        f_boundaryConditionX0(boundaryConditionX0),
        f_boundaryConditionX1(boundaryConditionX1),
        f_boundaryConditionY0(boundaryConditionY0),
        f_boundaryConditionY1(boundaryConditionY1)
        {}

        HeatProblemRectDBC2D(
            glm::dvec2&& domainOrigin,
            glm::dvec2&& domainSize,
            double mu,
            ScalarFunction2D&& initialCondition,
            ScalarFunction1D&& boundaryConditionX0,
            ScalarFunction1D&& boundaryConditionX1,
            ScalarFunction1D&& boundaryConditionY0,
            ScalarFunction1D&& boundaryConditionY1
        ):
        f_domainOrigin(std::move(domainOrigin)),
        f_domainSize(std::move(domainSize)),
        f_mu(mu),
        f_initialCondition(initialCondition),
        f_boundaryConditionX0(std::move(boundaryConditionX0)),
        f_boundaryConditionX1(std::move(boundaryConditionX1)),
        f_boundaryConditionY0(std::move(boundaryConditionY0)),
        f_boundaryConditionY1(std::move(boundaryConditionY1))
        {}
    };
}
