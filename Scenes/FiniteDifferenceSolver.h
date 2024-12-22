#pragma once

#include "HeatProblem.h"
#include "GridFunction.h"
#include "util/pcgsolver.h"

namespace FiniteDifferenceSolver {
    class FD_HPRDBC2D_Solver {
    private:
        // Problem
        HeatProblem::HeatProblemRectDBC2D f_problem;
        // State time
        double f_time;
    
    public:

        #pragma region Constructors

        FD_HPRDBC2D_Solver():
        f_problem(),
        f_time(0.)
        {}

        FD_HPRDBC2D_Solver(
            const HeatProblem::HeatProblemRectDBC2D& problem
        ):
        f_problem(problem),
        f_time(0.)
        {}

        FD_HPRDBC2D_Solver(
            HeatProblem::HeatProblemRectDBC2D&& problem
        ):
        f_problem(std::move(problem)),
        f_time(0.)
        {}

        #pragma endregion

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

        void enforceBoundaryConditionsOn(GridFunction::ScalarGridFunction2D& state) {
            // X0 BC
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

            // X1 BC
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

            //Y0 BC
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

            //Y1 BC
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

        unsigned getId(unsigned i, unsigned j, unsigned m) {
            return (i - 1) * (m - 2) + (j - 1);
        }

        void propagateStateImplicitOn(GridFunction::ScalarGridFunction2D& state, double delta) {
            // Update state time
            f_time += delta;
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
                        1.
                        + delta * f_problem.f_mu * 2 * (
                            1. / (state.getStepX() * state.getStepX())
                            + 1. / (state.getStepY() * state.getStepY())
                        );
                    A.set_element(id, id, majorValue);

                    // Set matrix values for (i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)
                    double minorValueX = -delta * f_problem.f_mu / (state.getStepX() * state.getStepX());
                    double minorValueY = -delta * f_problem.f_mu / (state.getStepY() * state.getStepY());
                    if (i != 1)                A.set_element(id, getId(i - 1, j, state.getM()), minorValueX);
                    if (i != state.getN() - 2) A.set_element(id, getId(i + 1, j, state.getM()), minorValueX);
                    if (j != 1)                A.set_element(id, getId(i, j - 1, state.getM()), minorValueY);
                    if (j != state.getM() - 2) A.set_element(id, getId(i, j + 1, state.getM()), minorValueY);

                    // Set RHS vector values
                    double rhsValue = stCp.getValue(i, j) + f_problem.f_sources(state.getPoint(i, j), f_time);
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
    };

    class FD_HPRDBC3D_Solver {
    private:
        // Problem
        HeatProblem::HeatProblemRectDBC3D f_problem;
        // State time
        double f_time;
    
    public:

        #pragma region Constructors

        FD_HPRDBC3D_Solver():
        f_problem(),
        f_time(0.)
        {}

        FD_HPRDBC3D_Solver(
            const HeatProblem::HeatProblemRectDBC3D& problem
        ):
        f_problem(problem),
        f_time(0.)
        {}

        FD_HPRDBC3D_Solver(
            HeatProblem::HeatProblemRectDBC3D&& problem
        ):
        f_problem(std::move(problem)),
        f_time(0.)
        {}

        #pragma endregion

        void setDiffusivity(double mu) {
            f_problem.f_mu = mu;
        }

        void setSources(const HeatProblem::ScalarTimeFunction3D& sources) {
            f_problem.setSources(sources);
        }

        void setSources(HeatProblem::ScalarTimeFunction3D&& sources) {
            f_problem.setSources(sources);
        }

        void setInitialConditionsOn(GridFunction::ScalarGridFunction3D& state) {
            for (unsigned i = 0; i < state.getN(); i++) {
                for (unsigned j = 0; j < state.getM(); j++) {
                    for (unsigned k = 0; k < state.getP(); k++) {
                        state.setValue(
                            i,
                            j,
                            k,
                            f_problem.f_initialCondition(
                                state.getPoint(i, j, k),
                                0.
                            )
                        );
                    }
                }
            }
        }

        void enforceBoundaryConditionsOn(GridFunction::ScalarGridFunction3D& state) {
            // X0 BC
            for (unsigned j = 0; j < state.getM(); j++) {
                for (unsigned k = 0; k < state.getP(); k++) {
                    state.setValue(
                        0,
                        j,
                        k,
                        f_problem.f_boundaryConditionX0(
                            glm::dvec2(state.getPoint(0, j, k).y, state.getPoint(0, j, k).z),
                            f_time
                        )
                    );
                }
            }

            // X1 BC
            for (unsigned j = 0; j < state.getM(); j++) {
                for (unsigned k = 0; k < state.getP(); k++) {
                    state.setValue(
                        state.getN() - 1,
                        j,
                        k,
                        f_problem.f_boundaryConditionX1(
                            glm::dvec2(state.getPoint(state.getN() - 1, j, k).y, state.getPoint(state.getN() - 1, j, k).z),
                            f_time
                        )
                    );
                }
            }

            //Y0 BC
            for (unsigned i = 0; i < state.getN(); i++) {
                for (unsigned k = 0; k < state.getP(); k++) {
                    state.setValue(
                        i,
                        0,
                        k,
                        f_problem.f_boundaryConditionY0(
                            glm::dvec2(state.getPoint(i, 0, k).x, state.getPoint(i, 0, k).z),
                            f_time
                        )
                    );
                }
            }

            //Y1 BC
            for (unsigned i = 0; i < state.getN(); i++) {
                for (unsigned k = 0; k < state.getP(); k++) {
                    state.setValue(
                        i,
                        state.getM() - 1,
                        k,
                        f_problem.f_boundaryConditionY1(
                            glm::dvec2(state.getPoint(i, state.getM() - 1, k).x, state.getPoint(i, state.getM() - 1, k).z),
                            f_time
                        )
                    );
                }
            }

            //Z0 BC
            for (unsigned i = 0; i < state.getN(); i++) {
                for (unsigned j = 0; j < state.getM(); j++) {
                    state.setValue(
                        i,
                        j,
                        0,
                        f_problem.f_boundaryConditionZ0(
                            glm::dvec2(state.getPoint(i, j, 0).x, state.getPoint(i, j, 0).y),
                            f_time
                        )
                    );
                }
            }

            //Z1 BC
            for (unsigned i = 0; i < state.getN(); i++) {
                for (unsigned j = 0; j < state.getM(); j++) {
                    state.setValue(
                        i,
                        j,
                        state.getP() - 1,
                        f_problem.f_boundaryConditionZ1(
                            glm::dvec2(state.getPoint(i, j, state.getP() - 1).x, state.getPoint(i, j, state.getP() - 1).y),
                            f_time
                        )
                    );
                }
            }
        }

        GridFunction::ScalarGridFunction3D getInitialState(unsigned n, unsigned m, unsigned p) {
            GridFunction::ScalarGridFunction3D initialState(
                n,
                m,
                p,
                f_problem.f_domainOrigin,
                f_problem.f_domainSize
            );
            // Set initial conditions
            setInitialConditionsOn(initialState);
            // Enforce boundary conditions
            enforceBoundaryConditionsOn(initialState);
            return initialState;
        }

        void propagateStateExplicitOn(GridFunction::ScalarGridFunction3D& state, double delta) {
            // Update state time
            f_time += delta;
            // Make currect state values copy
            GridFunction::IntegerGridScalarFunction3D stCp = state.getIntegerGridScalarFunction();
            
            // Update interior values
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    for (unsigned k = 1; k < state.getM() - 1; k++) {
                        double value = stCp.getValue(i, j, k)
                            + f_problem.f_sources(state.getPoint(i, j, k), f_time)
                            + f_problem.f_mu * delta * (
                                (stCp.getValue(i + 1, j, k) - 2 * stCp.getValue(i, j, k) + stCp.getValue(i - 1, j, k)) / (state.getStepX() * state.getStepX())
                                + (stCp.getValue(i, j + 1, k) - 2 * stCp.getValue(i, j, k) + stCp.getValue(i, j - 1, k)) / (state.getStepY() * state.getStepY())
                                + (stCp.getValue(i, j, k + 1) - 2 * stCp.getValue(i, j, k) + stCp.getValue(i, j, k - 1)) / (state.getStepZ() * state.getStepZ())
                            );
                        state.setValue(i, j, k, value);
                    }
                }
            }

            // Enforce boundary conditions
            enforceBoundaryConditionsOn(state);
        }

        unsigned getId(unsigned i, unsigned j, unsigned k, unsigned m, unsigned p) {
            return (i - 1) * (m - 2) * (p - 2) + (j - 1) * (p - 2) + (k - 1);
        }

        void propagateStateImplicitOn(GridFunction::ScalarGridFunction3D& state, double delta) {
            // Update state time
            f_time += delta;
            // Make currect state values copy
            GridFunction::IntegerGridScalarFunction3D stCp = state.getIntegerGridScalarFunction();

            // Fill system matrix and RHS vector
            SparseMatrix<double> A((state.getN() - 2) * (state.getM() - 2) * (state.getP() - 2), 7);
            A.zero();
            std::vector<double> rhs((state.getN() - 2) * (state.getM() - 2) * (state.getP() - 2));
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    for (unsigned k = 1; k < state.getP() - 1; k++) {
                        unsigned id = getId(i, j, k, state.getM(), state.getP());
                        // Set matrix value for id, (i,j,k)
                        double majorValue = 
                            1.
                            + delta * f_problem.f_mu * 2 * (
                                1. / (state.getStepX() * state.getStepX())
                                + 1. / (state.getStepY() * state.getStepY())
                                + 1. / (state.getStepZ() * state.getStepZ())
                            );
                        A.set_element(id, id, majorValue);

                        // Set matrix values for (i - 1, j, k), (i + 1, j, k), (i, j - 1, k), (i, j + 1, k), (i, j, k - 1), (i, j, k + 1)
                        double minorValueX = -delta * f_problem.f_mu / (state.getStepX() * state.getStepX());
                        double minorValueY = -delta * f_problem.f_mu / (state.getStepY() * state.getStepY());
                        double minorValueZ = -delta * f_problem.f_mu / (state.getStepZ() * state.getStepZ());
                        if (i != 1)                A.set_element(id, getId(i - 1, j, k, state.getM(), state.getP()), minorValueX);
                        if (i != state.getN() - 2) A.set_element(id, getId(i + 1, j, k, state.getM(), state.getP()), minorValueX);
                        if (j != 1)                A.set_element(id, getId(i, j - 1, k, state.getM(), state.getP()), minorValueY);
                        if (j != state.getM() - 2) A.set_element(id, getId(i, j + 1, k, state.getM(), state.getP()), minorValueY);
                        if (k != 1)                A.set_element(id, getId(i, j, k - 1, state.getM(), state.getP()), minorValueZ);
                        if (k != state.getP() - 2) A.set_element(id, getId(i, j, k + 1, state.getM(), state.getP()), minorValueZ);

                        // Set RHS vector values
                        double rhsValue = stCp.getValue(i, j, k) + f_problem.f_sources(state.getPoint(i, j, k), f_time);
                        if (i == 1)                rhsValue += -minorValueX * stCp.getValue(i - 1, j, k);
                        if (i == state.getN() - 2) rhsValue += -minorValueX * stCp.getValue(i + 1, j, k);
                        if (j == 1)                rhsValue += -minorValueY * stCp.getValue(i, j - 1, k);
                        if (j == state.getM() - 2) rhsValue += -minorValueY * stCp.getValue(i, j + 1, k);
                        if (k == 1)                rhsValue += -minorValueZ * stCp.getValue(i, j, k - 1);
                        if (k == state.getP() - 2) rhsValue += -minorValueZ * stCp.getValue(i, j, k + 1);
                        rhs[id] = rhsValue;
                    }
                }
            }
            
            // Solve system
            SparsePCGSolver<double> SPCGS;
            std::vector<double> result((state.getN() - 2) * (state.getM() - 2) * (state.getP() - 2));
            double residual;
            int iterations;
            SPCGS.solve(A, rhs, result, residual, iterations);

            // Update values for internal points
            for (unsigned i = 1; i < state.getN() - 1; i++) {
                for (unsigned j = 1; j < state.getM() - 1; j++) {
                    for (unsigned k = 1; k < state.getP() - 1; k++) {
                        unsigned id = getId(i, j, k, state.getM(), state.getP());
                        state.setValue(i, j, k, result[id]);
                    }
                }
            }

            // Enforce boundary conditions
            enforceBoundaryConditionsOn(state);
        }
    };
}