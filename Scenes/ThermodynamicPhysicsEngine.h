#pragma once

#include "HeatProblem.h"
#include "GridFunction.h"
#include "util/pcgsolver.h"

namespace Physics {
    /** @brief
     * Heat solver type enumeration.
     */
    enum class SolverType {
        EXPLICIT,
        IMPLICIT
    };


    /**
     * Responsible for numerical integration of thermodynamic properties of the system
     * and handles collisions.
     */
    class ThermodynamicPhysicsEngine {
    private:
        /// @brief Heat problem.
        HeatProblem::HeatProblemRectDBC2D f_problem;
        /// @brief Current state elapsed time.
        double f_time;
    public:
        SolverType f_solverType;
    public:

        #pragma region Constructors

        ThermodynamicPhysicsEngine():
        f_problem(),
        f_time(0.),
        f_solverType(SolverType::IMPLICIT)
        {}

        ThermodynamicPhysicsEngine(
            const HeatProblem::HeatProblemRectDBC2D& problem
        ):
        f_problem(problem),
        f_time(0.),
        f_solverType(SolverType::IMPLICIT)
        {}

        ThermodynamicPhysicsEngine(
            HeatProblem::HeatProblemRectDBC2D&& problem
        ):
        f_problem(std::move(problem)),
        f_time(0.),
        f_solverType(SolverType::IMPLICIT)
        {}

        #pragma endregion

        #pragma region Getters/Setters

        void setDiffusivity(double mu) {
            f_problem.f_mu = mu;
        }

        void setSources(const HeatProblem::ScalarTimeFunction2D& sources) {
            f_problem.setSources(sources);
        }

        void setSources(HeatProblem::ScalarTimeFunction2D&& sources) {
            f_problem.setSources(sources);
        }

        void setInitialConditionsOn(GridFunction::ScalarGridFunction2D& state) {
            for (unsigned i = 0; i < state.getN(); i++) {
                for (unsigned j = 0; j < state.getM(); j++) {
                    state.setValue(
                        i,
                        j,
                        f_problem.f_initialCondition(
                            state.getPoint(i, j),
                            0.
                        )
                    );
                }
            }
        }

        #pragma endregion

        /// @brief Enforces Dirichlet boundary conditions on a state.
        /// @param state State to inforce Dirichlet boundary conditions on.
        void enforceBoundaryConditionsOn(GridFunction::ScalarGridFunction2D& state) {
            // X0 BC
            if (f_problem.f_BCX0Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                for (unsigned j = 0; j < state.getM(); j++) {
                    state.setValue(
                        0,
                        j,
                        f_problem.f_boundaryConditionX0(
                            state.getPoint(0, j).y,
                            f_time
                        )
                    );
                }
            }

            // X1 BC
            if (f_problem.f_BCX1Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                for (unsigned j = 0; j < state.getM(); j++) {
                    state.setValue(
                        state.getN() - 1,
                        j,
                        f_problem.f_boundaryConditionX1(
                            state.getPoint(state.getN() - 1, j).y,
                            f_time
                        )
                    );
                }
            }

            //Y0 BC
            if (f_problem.f_BCY0Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                for (unsigned i = 0; i < state.getN(); i++) {
                    state.setValue(
                        i,
                        0,
                        f_problem.f_boundaryConditionY0(
                            state.getPoint(i, 0).x,
                            f_time
                        )
                    );
                }
            }

            //Y1 BC
            if (f_problem.f_BCY1Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                for (unsigned i = 0; i < state.getN(); i++) {
                    state.setValue(
                        i,
                        state.getM() - 1,
                        f_problem.f_boundaryConditionY1(
                            state.getPoint(i, state.getM() - 1).x,
                            f_time
                        )
                    );
                }
            }
        }

        /// @brief Returns a grid function for the specified problem f_problem with given grid dimensions.
        /// @param n Grid dimension along Ox.
        /// @param m Grid dimension along Oy.
        /// @return ScalarGridFunction2D initial state of the system.
        GridFunction::ScalarGridFunction2D getInitialState(unsigned n, unsigned m) {
            GridFunction::ScalarGridFunction2D initialState(
                n,
                m,
                f_problem.f_domainOrigin,
                f_problem.f_domainSize
            );
            // Set initial conditions
            setInitialConditionsOn(initialState);
            // Enforce boundary conditions
            enforceBoundaryConditionsOn(initialState);
            return initialState;
        }

        /// @brief Propagates a state forward in time using an explicit integration scheme.
        /// @param state Thermodynamical system state to propagate.
        /// @param delta Time step.
        void propagateStateExplicitOn(GridFunction::ScalarGridFunction2D& state, double delta) {
            // Update state time
            f_time += delta;
            // Make currect state values copy
            GridFunction::IntegerGridScalarFunction2D stCp = state.getIntegerGridScalarFunction();
            
            // Update interior values
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    double value = stCp.getValue(i, j)
                        + f_problem.f_sources(state.getPoint(i, j), f_time)
                        + f_problem.f_mu * delta * (
                            (stCp.getValue(i + 1, j) - 2 * stCp.getValue(i, j) + stCp.getValue(i - 1, j)) / (state.getStepX() * state.getStepX())
                            + (stCp.getValue(i, j + 1) - 2 * stCp.getValue(i, j) + stCp.getValue(i, j - 1)) / (state.getStepY() * state.getStepY())
                        );
                    state.setValue(i, j, value);
                }
            }

            // Enforce boundary conditions
            enforceBoundaryConditionsOn(state);
        }

        /// @brief Returns a global enumeration id for a given grid vertex.
        /// @param i Vertex index along Ox.
        /// @param j Vertex index along Oy.
        /// @param m Grid dimension along Oy.
        /// @return Global enumeration id.
        unsigned getId(unsigned i, unsigned j, unsigned m) {
            return (i - 1) * (m - 2) + (j - 1);
        }

        /// @brief Propagates a state forward in time using an implicit integration scheme.
        /// @param state Thermodynamical system state to propagate.
        /// @param delta Time step.
        void propagateStateImplicitOn(GridFunction::ScalarGridFunction2D& state, double delta) {
            // Update state time
            f_time += delta;
            // Make currect state values copy
            GridFunction::IntegerGridScalarFunction2D stCp = state.getIntegerGridScalarFunction();

            // Fill system matrix and RHS vector
            SparseMatrix<double> A((state.getN() - 2) * (state.getM() - 2), 5);
            A.zero();
            std::vector<double> rhs((state.getN() - 2) * (state.getM() - 2));
            double majorValue = 
                1.
                + delta * f_problem.f_mu * 2 * (
                    1. / (state.getStepX() * state.getStepX())
                    + 1. / (state.getStepY() * state.getStepY())
                );
            double minorValueX = -delta * f_problem.f_mu / (state.getStepX() * state.getStepX());
            double minorValueY = -delta * f_problem.f_mu / (state.getStepY() * state.getStepY());
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    // Get equation id
                    unsigned id = getId(i, j, state.getM());

                    // Precalculate matrix value for id, (i,j)
                    double ijV = majorValue;

                    // Set matrix values for (i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)
                    if (i != 1)                A.set_element(id, getId(i - 1, j, state.getM()), minorValueX);
                    if (i != state.getN() - 2) A.set_element(id, getId(i + 1, j, state.getM()), minorValueX);
                    if (j != 1)                A.set_element(id, getId(i, j - 1, state.getM()), minorValueY);
                    if (j != state.getM() - 2) A.set_element(id, getId(i, j + 1, state.getM()), minorValueY);

                    // Precalculate rhs vector value
                    double rhsValue = stCp.getValue(i, j) + f_problem.f_sources(state.getPoint(i, j), f_time);

                    // Account for boundary conditions
                    // If at X0 boundary
                    if (i == 1) {
                        // For Dirichlet BC
                        if (f_problem.f_BCX0Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                            // Account for i = 0 node value in rhs
                            rhsValue += -minorValueX * stCp.getValue(i - 1, j);
                        }
                        // For Neuman BC
                        else if (f_problem.f_BCX0Type == HeatProblem::BoundaryConditionType::NEUMAN) {
                            // Account for flux value in rhs
                            rhsValue += minorValueX * state.getStepX() * f_problem.f_boundaryConditionX0(state.getPoint(i, j).y, f_time);
                            // Account fot finite difference in the system matrix for (i, j)
                            ijV += -delta * f_problem.f_mu / (state.getStepX() * state.getStepX());
                        }
                    }
                    // If at X1
                    if (i == state.getN() - 2) {
                        // For Dirichlet BC
                        if (f_problem.f_BCX1Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                            // Account for i = state.getN() - 1 node value in rhs
                            rhsValue += -minorValueX * stCp.getValue(i + 1, j);
                        }
                        // For Neuman BC
                        else if (f_problem.f_BCX1Type == HeatProblem::BoundaryConditionType::NEUMAN) {
                            // Account for flux value in rhs
                            rhsValue += minorValueX * state.getStepX() * f_problem.f_boundaryConditionX1(state.getPoint(i, j).y, f_time);
                            // Account fot finite difference in the system matrix for (i, j)
                            ijV += -delta * f_problem.f_mu / (state.getStepX() * state.getStepX());
                        }
                    }
                    // Analogous for Y0 and Y1
                    if (j == 1) {
                        if (f_problem.f_BCY0Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                            rhsValue += -minorValueY * stCp.getValue(i, j - 1);
                        }
                        else if (f_problem.f_BCY0Type == HeatProblem::BoundaryConditionType::NEUMAN) {
                            rhsValue += minorValueY * state.getStepX() * f_problem.f_boundaryConditionY0(state.getPoint(i, j).x, f_time);
                            ijV += -delta * f_problem.f_mu / (state.getStepY() * state.getStepY());
                        }
                    }
                    if (j == state.getM() - 2) {
                        if (f_problem.f_BCY1Type == HeatProblem::BoundaryConditionType::DIRICHLET) {
                            rhsValue += -minorValueY * stCp.getValue(i, j + 1);
                        }
                        else if (f_problem.f_BCY1Type == HeatProblem::BoundaryConditionType::NEUMAN) {
                            rhsValue += minorValueY * state.getStepX() * f_problem.f_boundaryConditionY1(state.getPoint(i, j).x, f_time);
                            ijV += -delta * f_problem.f_mu / (state.getStepY() * state.getStepY());
                        }
                    }
                    
                    // Set rhs vector and main matrix values
                    rhs[id] = rhsValue;
                    A.set_element(id, id, ijV);
                }
            }
            
            // Solve system
            SparsePCGSolver<double> SPCGS;
            std::vector<double> result((state.getN() - 2) * (state.getM() - 2));
            double residual;
            int iterations;
            SPCGS.solve(A, rhs, result, residual, iterations);

            // Update values for internal points
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    unsigned id = getId(i, j, state.getM());
                    state.setValue(i, j, result[id]);
                }
            }

            // Enforce boundary conditions
            enforceBoundaryConditionsOn(state);
        }
    };
}