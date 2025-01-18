#pragma once

#include <functional>
#include "Renderer.h"

/** @brief
 * Namespace of heat problem definition related objects.
 */
namespace HeatProblem {
    /// @brief Scalar function with a 2D spatial pratmeter and a temporal parameter.
    typedef std::function<double(glm::dvec2, double)> ScalarTimeFunction2D;
    /// @brief Scalar function with a spatial pratmeter and a temporal parameter.
    typedef std::function<double(double, double)> ScalarTimeFunction1D;

    enum class BoundaryConditionType {
        DIRICHLET,
        NEUMAN
    };

    /** @brief
     * Heat problem on a rectangle strucure.
     * Encompases all properties of a mathematical definition of a heat BVP on a rectangle.
     */
    struct HeatProblemRectDBC2D {
        // Domain
        /// @brief Domain origin.
        glm::dvec2 f_domainOrigin;
        /// @brief Domain size.
        glm::dvec2 f_domainSize;

        /// @brief Thermal diffusivity of the material.
        double f_mu;

        /// @brief Heat sources function.
        ScalarTimeFunction2D f_sources;
        
        /// @brief Initial condition function.
        ScalarTimeFunction2D f_initialCondition;

        /// @brief Boundary condition function for smaller x domain border.
        ScalarTimeFunction1D f_boundaryConditionX0;
        /// @brief Boundary condition function for bigger x domain border.
        ScalarTimeFunction1D f_boundaryConditionX1;
        /// @brief Boundary condition function for smaller y domain border.
        ScalarTimeFunction1D f_boundaryConditionY0;
        /// @brief Boundary condition function for bigger y domain border.
        ScalarTimeFunction1D f_boundaryConditionY1;
        
        /// @brief Boundary condition type flag for smaller x domain border.
        BoundaryConditionType f_BCX0Type;
        /// @brief Boundary condition type flag for bigger x domain border.
        BoundaryConditionType f_BCX1Type;
        /// @brief Boundary condition type flag for smaller y domain border.
        BoundaryConditionType f_BCY0Type;
        /// @brief Boundary condition type flag for bigger y domain border.
        BoundaryConditionType f_BCY1Type;

        #pragma region Constructors

        HeatProblemRectDBC2D():
        f_domainOrigin(0.),
        f_domainSize(0.),
        f_mu(0.),
        f_sources(nullptr),
        f_initialCondition(nullptr),
        f_boundaryConditionX0(nullptr),
        f_boundaryConditionX1(nullptr),
        f_boundaryConditionY0(nullptr),
        f_boundaryConditionY1(nullptr),
        f_BCX0Type(BoundaryConditionType::DIRICHLET),
        f_BCX1Type(BoundaryConditionType::DIRICHLET),
        f_BCY0Type(BoundaryConditionType::DIRICHLET),
        f_BCY1Type(BoundaryConditionType::DIRICHLET)
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
            const ScalarTimeFunction1D& boundaryConditionY1,
            const BoundaryConditionType& BCX0Type,
            const BoundaryConditionType& BCX1Type,
            const BoundaryConditionType& BCY0Type,
            const BoundaryConditionType& BCY1Type
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
        f_BCX0Type(BCX0Type),
        f_BCX1Type(BCX1Type),
        f_BCY0Type(BCY0Type),
        f_BCY1Type(BCY1Type)
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
            ScalarTimeFunction1D&& boundaryConditionY1,
            BoundaryConditionType BCX0Type,
            BoundaryConditionType BCX1Type,
            BoundaryConditionType BCY0Type,
            BoundaryConditionType BCY1Type
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
        f_BCX0Type(BCX0Type),
        f_BCX1Type(BCX1Type),
        f_BCY0Type(BCY0Type),
        f_BCY1Type(BCY1Type)
        {}

        #pragma endregion

        void setSources(const ScalarTimeFunction2D& sources) {
            f_sources = sources;
        }

        void setSources(ScalarTimeFunction2D&& sources) {
            f_sources = std::move(sources);
        }
    };
}