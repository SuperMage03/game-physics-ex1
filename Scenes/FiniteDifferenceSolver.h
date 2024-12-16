#pragma once

#include "HeatProblem.h"
#include "GridFunction.h"
#include "util/pcgsolver.h"

namespace FiniteDifferenceSolver {
    class FD_HPRDBC2D_Solver {
    private:
        // Problem
        HeatProblem::HeatProblemRectDBC2D f_problem;
    
    public:

        #pragma region Constructors
        FD_HPRDBC2D_Solver():
        f_problem()
        {}

        FD_HPRDBC2D_Solver(
            const HeatProblem::HeatProblemRectDBC2D& problem
        ):
        f_problem(problem)
        {}

        FD_HPRDBC2D_Solver(
            HeatProblem::HeatProblemRectDBC2D&& problem
        ):
        f_problem(std::move(problem))
        {}

        #pragma endregion

        void setInitialConditionsOn(GridFunction::ScalarGridFunction2D& gridFunction) {
            for (unsigned i = 0; i < gridFunction.getN(); i++) {
                for (unsigned j = 0; j < gridFunction.getM(); j++) {
                    gridFunction.setValue(
                        i,
                        j,
                        f_problem.f_initialCondition(
                            gridFunction.getPoint(i, j)
                        )
                    );
                }
            }
        }

        void enforceBoundaryConditionsOn(GridFunction::ScalarGridFunction2D& gridFunction) {
            // X0 BC
            for (unsigned j = 0; j < gridFunction.getM(); j++) {
                gridFunction.setValue(
                    0,
                    j,
                    f_problem.f_boundaryConditionX0(
                        gridFunction.getPoint(0, j).y
                    )
                );
            }

            // X1 BC
            for (unsigned j = 0; j < gridFunction.getM(); j++) {
                gridFunction.setValue(
                    gridFunction.getN() - 1,
                    j,
                    f_problem.f_boundaryConditionX1(
                        gridFunction.getPoint(gridFunction.getN() - 1, j).y
                    )
                );
            }

            //Y0 BC
            for (unsigned i = 0; i < gridFunction.getN(); i++) {
                gridFunction.setValue(
                    i,
                    0,
                    f_problem.f_boundaryConditionY0(
                        gridFunction.getPoint(i, 0).x
                    )
                );
            }

            //Y1 BC
            for (unsigned i = 0; i < gridFunction.getN(); i++) {
                gridFunction.setValue(
                    i,
                    gridFunction.getM() - 1,
                    f_problem.f_boundaryConditionY1(
                        gridFunction.getPoint(i, gridFunction.getM() - 1).x
                    )
                );
            }
        }

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

        void propagateStateExplicitOn(GridFunction::ScalarGridFunction2D& state, double delta) {
            // Make currect state values copy
            GridFunction::IntegerGridScalarFunction2D stCp = state.getIntegerGridScalarFunction();

            // Update interior values
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    double value = stCp.getValue(i, j)
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

        unsigned getId(unsigned i, unsigned j, unsigned m) {
            return (i - 1) * (m - 2) + (j - 1);
        }

        void propagateStateImplicitOn(GridFunction::ScalarGridFunction2D& state, double delta) {
            // Make currect state values copy
            GridFunction::IntegerGridScalarFunction2D stCp = state.getIntegerGridScalarFunction();

            // Fill system matrix and RHS vector
            SparseMatrix<double> A((state.getN() - 2) * (state.getM() - 2), 5);
            A.zero();
            std::vector<double> rhs((state.getN() - 2) * (state.getM() - 2));
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    unsigned id = getId(i, j, state.getM());
                    // Set matrix value for id, (i,j)
                    double majorValue = 
                        1. / delta
                        + f_problem.f_mu * 2 * (
                            1. / (state.getStepX() * state.getStepX())
                            + 1. / (state.getStepY() * state.getStepY())
                        );
                    A.set_element(id, id, majorValue);

                    // Set matrix values for (i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)
                    double minorValueX = -f_problem.f_mu / (state.getStepX() * state.getStepX());
                    double minorValueY = -f_problem.f_mu / (state.getStepY() * state.getStepY());
                    if (i != 1)                A.set_element(id, getId(i - 1, j, state.getM()), minorValueX);
                    if (i != state.getN() - 2) A.set_element(id, getId(i + 1, j, state.getM()), minorValueX);
                    if (j != 1)                A.set_element(id, getId(i, j - 1, state.getM()), minorValueY);
                    if (j != state.getM() - 2) A.set_element(id, getId(i, j + 1, state.getM()), minorValueY);

                    // Set RHS vector values
                    double rhsValue = stCp.getValue(i, j) / delta;
                    if (i == 1)                rhsValue += -minorValueX * stCp.getValue(i - 1, j);
                    if (i == state.getN() - 2) rhsValue += -minorValueX * stCp.getValue(i + 1, j);
                    if (j == 1)                rhsValue += -minorValueY * stCp.getValue(i, j - 1);
                    if (j == state.getM() - 2) rhsValue += -minorValueY * stCp.getValue(i, j + 1);
                    rhs[id] = rhsValue;
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

        // // TEST, NEGATED EQUATION
        // void propagateStateImplicitOn(GridFunction::ScalarGridFunction2D& state, double delta) {
        //     // Make currect state values copy
        //     GridFunction::IntegerGridScalarFunction2D stCp = state.getIntegerGridScalarFunction();

        //     // Fill system matrix and RHS vector
        //     SparseMatrix<double> A((state.getN() - 2) * (state.getM() - 2), 5);
        //     A.zero();
        //     std::vector<double> rhs((state.getN() - 2) * (state.getM() - 2));
        //     for (unsigned i = 1; i < state.getN() - 1; i++) {
        //         for (unsigned j = 1; j < state.getM() - 1; j++) {
        //             unsigned id = getId(i, j, state.getM());
        //             // Set matrix value for id, (i,j)
        //             double majorValue = 
        //                 1. / delta
        //                 + f_problem.f_mu * 2 * (
        //                     1. / (state.getStepX() * state.getStepX())
        //                     + 1. / (state.getStepY() * state.getStepY())
        //                 );
        //             majorValue *= -1.;
        //             A.set_element(id, id, majorValue);

        //             // Set matrix values for (i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)
        //             double minorValueX = -f_problem.f_mu / (state.getStepX() * state.getStepX());
        //             double minorValueY = -f_problem.f_mu / (state.getStepY() * state.getStepY());
        //             minorValueX *= -1.;
        //             minorValueY *= -1.;
        //             if (i != 1)                A.set_element(id, getId(i - 1, j, state.getM()), minorValueX);
        //             if (i != state.getN() - 2) A.set_element(id, getId(i + 1, j, state.getM()), minorValueX);
        //             if (j != 1)                A.set_element(id, getId(i, j - 1, state.getM()), minorValueY);
        //             if (j != state.getM() - 2) A.set_element(id, getId(i, j + 1, state.getM()), minorValueY);

        //             // Set RHS vector values
        //             double rhsValue = stCp.getValue(i, j) / delta;
        //             rhsValue *= -1.;
        //             if (i == 1)                rhsValue += -minorValueX * stCp.getValue(i - 1, j);
        //             if (i == state.getN() - 2) rhsValue += -minorValueX * stCp.getValue(i + 1, j);
        //             if (j == 1)                rhsValue += -minorValueY * stCp.getValue(i, j - 1);
        //             if (j == state.getM() - 2) rhsValue += -minorValueY * stCp.getValue(i, j + 1);
        //             rhs[id] = rhsValue;
        //         }
        //     }
            
        //     // Solve system
        //     SparsePCGSolver<double> SPCGS;
        //     std::vector<double> result((state.getN() - 2) * (state.getM() - 2));
        //     double residual;
        //     int iterations;
        //     SPCGS.solve(A, rhs, result, residual, iterations);

        //     // Update values for internal points
        //     for (unsigned i = 1; i < state.getN() - 1; i++) {
        //         for (unsigned j = 1; j < state.getM() - 1; j++) {
        //             unsigned id = getId(i, j, state.getM());
        //             state.setValue(i, j, result[id]);
        //         }
        //     }

        //     // Enforce boundary conditions
        //     enforceBoundaryConditionsOn(state);
        // }
    };
}