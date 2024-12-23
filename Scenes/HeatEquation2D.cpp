#include "HeatEquation2D.hpp"
#include <cmath>
#include <util/pcgsolver.h>

void HeatEquation2D::simulateStepEulerExplicit(const float& deltaTime) {
    const Grid2D gridCopy{m_grid};

    for (unsigned int row = 0; row < m_grid.getGridRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getGridColSize(); col++) {
            float delta = m_diffusivity * (((gridCopy.getPlotValueAtPosition(row+1, col) - 2*gridCopy.getPlotValueAtPosition(row, col) + gridCopy.getPlotValueAtPosition(row-1, col)) / powf(gridCopy.getDeltaX(), 2.0f)) + 
                                 ((gridCopy.getPlotValueAtPosition(row, col+1) - 2*gridCopy.getPlotValueAtPosition(row, col) + gridCopy.getPlotValueAtPosition(row, col-1)) / powf(gridCopy.getDeltaY(), 2.0f))) * deltaTime;
            m_grid.setPlotValueAtPosition(row, col, gridCopy.getPlotValueAtPosition(row, col) + delta);
        }
    }
}

void HeatEquation2D::simulateStepEulerImplicit(const float& deltaTime) {
    const int matrixDimension = m_grid.getGridRowSize() * m_grid.getGridColSize();

    // PCG Solvers and the out value containers
    SparsePCGSolver<float> sparsePCGSolver;
    std::vector<float> result(matrixDimension);
    float residual;
    int iterations;

    // Setting up the matrix and rhs
    SparseMatrix<float> matrix(matrixDimension, 5);
    std::vector<float> rhs(matrixDimension);

    const float lambdaX = m_diffusivity * deltaTime / powf(m_grid.getDeltaX(), 2.0f);
    const float lambdaY = m_diffusivity * deltaTime / powf(m_grid.getDeltaY(), 2.0f);

    for (unsigned int matrixRow = 0; matrixRow < matrixDimension; matrixRow++) {
        glm::vec2 gridPosition = m_grid.unflattenFunction(matrixRow);
        int gridRow = gridPosition.x;
        int gridCol = gridPosition.y;

        // Populate RHS
        rhs[matrixRow] = m_grid.getPlotValueAtPosition(gridRow, gridCol);

        int matrixCol_ImJ = m_grid.flattenFunction(gridRow-1, gridCol);
        int matrixCol_IJm = m_grid.flattenFunction(gridRow, gridCol-1);
        int matrixCol_IJ = matrixRow;
        int matrixCol_IJp = m_grid.flattenFunction(gridRow, gridCol+1);
        int matrixCol_IpJ = m_grid.flattenFunction(gridRow+1, gridCol);

        if (gridRow-1 < 0) {
            rhs[matrixRow] += lambdaX * m_grid.getPlotValueAtPosition(gridRow-1, gridCol);
        }
        else {
            matrix.set_element(matrixRow, matrixCol_ImJ, -lambdaX);
        }

        if (gridCol-1 < 0) {
            rhs[matrixRow] += lambdaY * m_grid.getPlotValueAtPosition(gridRow, gridCol-1);
        }
        else {
            matrix.set_element(matrixRow, matrixCol_IJm, -lambdaY);
        }

        matrix.set_element(matrixRow, matrixCol_IJ, 1 + 2*lambdaX + 2*lambdaY);

        if (gridCol+1 >= m_grid.getGridColSize()) {
            rhs[matrixRow] += lambdaY * m_grid.getPlotValueAtPosition(gridRow, gridCol+1);
        }
        else {
            matrix.set_element(matrixRow, matrixCol_IJp, -lambdaY);
        }

        if (gridRow+1 >= m_grid.getGridRowSize()) {
            rhs[matrixRow] += lambdaX * m_grid.getPlotValueAtPosition(gridRow+1, gridCol);
        }
        else {
            matrix.set_element(matrixRow, matrixCol_IpJ, -lambdaX);
        }
    }

    // Solve
    sparsePCGSolver.solve(matrix, rhs, result, residual, iterations);

    // Write result to grid
    for (unsigned int row = 0; row < m_grid.getGridRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getGridColSize(); col++) {
            m_grid.setPlotValueAtPosition(row, col, result[m_grid.flattenFunction(row, col)]);
        }
    }
}

HeatEquation2D::HeatEquation2D(Grid2D &grid, const float& diffusivity, const IntegrationMode &integrationMode): m_grid{grid}, m_diffusivity{diffusivity}, m_integrationMode{integrationMode} {}

void HeatEquation2D::setDiffusivity(const float &diffusivity) {
    m_diffusivity = diffusivity;
}

void HeatEquation2D::simulateStep(const float& deltaTime) {
    switch (m_integrationMode) {
    case HeatEquation2D::IntegrationMode::EULER_EXPLICIT:
        simulateStepEulerExplicit(deltaTime);
        break;
    case HeatEquation2D::IntegrationMode::EULER_IMPLICIT:
        simulateStepEulerImplicit(deltaTime);
        break;
    default:
        break;
    }
    m_grid.setTime(m_grid.getTime() + deltaTime);
}
