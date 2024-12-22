#pragma once

#include <functional>
#include "Renderer.h"

namespace HeatProblem {
    typedef std::function<double(glm::dvec2, double)> ScalarTimeFunction2D;
    typedef std::function<double(double, double)> ScalarTimeFunction1D;

    struct HeatProblemRectDBC2D {
        // Domain
        // Origin point
        glm::dvec2 f_domainOrigin;
        // Size
        glm::dvec2 f_domainSize;

        // Thermal diffusivity
        double f_mu;

        // Heat sources function
        ScalarTimeFunction2D f_sources;
        
        // Initial condition
        ScalarTimeFunction2D f_initialCondition;

        // Boundary conditions
        ScalarTimeFunction1D f_boundaryConditionX0;
        ScalarTimeFunction1D f_boundaryConditionX1;
        ScalarTimeFunction1D f_boundaryConditionY0;
        ScalarTimeFunction1D f_boundaryConditionY1;

        HeatProblemRectDBC2D():
        f_domainOrigin(0.),
        f_domainSize(0.),
        f_mu(0.),
        f_sources(nullptr),
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
            const ScalarTimeFunction2D& sources,
            const ScalarTimeFunction2D& initialCondition,
            const ScalarTimeFunction1D& boundaryConditionX0,
            const ScalarTimeFunction1D& boundaryConditionX1,
            const ScalarTimeFunction1D& boundaryConditionY0,
            const ScalarTimeFunction1D& boundaryConditionY1
        ):
        f_domainOrigin(domainOrigin),
        f_domainSize(domainSize),
        f_mu(mu),
        f_sources(sources),
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
            ScalarTimeFunction2D&& sources,
            ScalarTimeFunction2D&& initialCondition,
            ScalarTimeFunction1D&& boundaryConditionX0,
            ScalarTimeFunction1D&& boundaryConditionX1,
            ScalarTimeFunction1D&& boundaryConditionY0,
            ScalarTimeFunction1D&& boundaryConditionY1
        ):
        f_domainOrigin(std::move(domainOrigin)),
        f_domainSize(std::move(domainSize)),
        f_mu(mu),
        f_sources(std::move(sources)),
        f_initialCondition(initialCondition),
        f_boundaryConditionX0(std::move(boundaryConditionX0)),
        f_boundaryConditionX1(std::move(boundaryConditionX1)),
        f_boundaryConditionY0(std::move(boundaryConditionY0)),
        f_boundaryConditionY1(std::move(boundaryConditionY1))
        {}

        void setSources(const ScalarTimeFunction2D& sources) {
            f_sources = sources;
        }

        void setSources(ScalarTimeFunction2D&& sources) {
            f_sources = std::move(sources);
        }
    };

    typedef std::function<double(glm::dvec3, double)> ScalarTimeFunction3D;

    struct HeatProblemRectDBC3D {
        // Domain
        // Origin point
        glm::dvec3 f_domainOrigin;
        // Size
        glm::dvec3 f_domainSize;

        // Thermal diffusivity
        double f_mu;

        // Heat sources function
        ScalarTimeFunction3D f_sources;
        
        // Initial condition
        ScalarTimeFunction3D f_initialCondition;

        // Boundary conditions
        ScalarTimeFunction2D f_boundaryConditionX0;
        ScalarTimeFunction2D f_boundaryConditionX1;
        ScalarTimeFunction2D f_boundaryConditionY0;
        ScalarTimeFunction2D f_boundaryConditionY1;
        ScalarTimeFunction2D f_boundaryConditionZ0;
        ScalarTimeFunction2D f_boundaryConditionZ1;

        HeatProblemRectDBC3D():
        f_domainOrigin(0.),
        f_domainSize(0.),
        f_mu(0.),
        f_sources(nullptr),
        f_initialCondition(nullptr),
        f_boundaryConditionX0(nullptr),
        f_boundaryConditionX1(nullptr),
        f_boundaryConditionY0(nullptr),
        f_boundaryConditionY1(nullptr),
        f_boundaryConditionZ0(nullptr),
        f_boundaryConditionZ1(nullptr)
        {}

        HeatProblemRectDBC3D(
            const glm::dvec3& domainOrigin,
            const glm::dvec3& domainSize,
            double mu,
            const ScalarTimeFunction3D& sources,
            const ScalarTimeFunction3D& initialCondition,
            const ScalarTimeFunction2D& boundaryConditionX0,
            const ScalarTimeFunction2D& boundaryConditionX1,
            const ScalarTimeFunction2D& boundaryConditionY0,
            const ScalarTimeFunction2D& boundaryConditionY1,
            const ScalarTimeFunction2D& boundaryConditionZ0,
            const ScalarTimeFunction2D& boundaryConditionZ1
        ):
        f_domainOrigin(domainOrigin),
        f_domainSize(domainSize),
        f_mu(mu),
        f_sources(sources),
        f_initialCondition(initialCondition),
        f_boundaryConditionX0(boundaryConditionX0),
        f_boundaryConditionX1(boundaryConditionX1),
        f_boundaryConditionY0(boundaryConditionY0),
        f_boundaryConditionY1(boundaryConditionY1),
        f_boundaryConditionZ0(boundaryConditionZ0),
        f_boundaryConditionZ1(boundaryConditionZ1)
        {}

        HeatProblemRectDBC3D(
            glm::dvec3&& domainOrigin,
            glm::dvec3&& domainSize,
            double mu,
            ScalarTimeFunction3D&& sources,
            ScalarTimeFunction3D&& initialCondition,
            ScalarTimeFunction2D&& boundaryConditionX0,
            ScalarTimeFunction2D&& boundaryConditionX1,
            ScalarTimeFunction2D&& boundaryConditionY0,
            ScalarTimeFunction2D&& boundaryConditionY1,
            ScalarTimeFunction2D&& boundaryConditionZ0,
            ScalarTimeFunction2D&& boundaryConditionZ1
        ):
        f_domainOrigin(std::move(domainOrigin)),
        f_domainSize(std::move(domainSize)),
        f_mu(mu),
        f_sources(std::move(sources)),
        f_initialCondition(initialCondition),
        f_boundaryConditionX0(std::move(boundaryConditionX0)),
        f_boundaryConditionX1(std::move(boundaryConditionX1)),
        f_boundaryConditionY0(std::move(boundaryConditionY0)),
        f_boundaryConditionY1(std::move(boundaryConditionY1)),
        f_boundaryConditionZ0(std::move(boundaryConditionZ0)),
        f_boundaryConditionZ1(std::move(boundaryConditionZ1))
        {}

        void setSources(const ScalarTimeFunction3D& sources) {
            f_sources = sources;
        }

        void setSources(ScalarTimeFunction3D&& sources) {
            f_sources = std::move(sources);
        }
    };
}
