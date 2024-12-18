#include "HeatEquation.h"
#include <cmath>
#include <util/pcgsolver.h>

void HeatEquation::simulateStepEulerExplicit(const float& deltaTime) {
    const Grid gridCopy{m_grid};

    for (unsigned int row = 0; row < m_grid.getGridRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getGridColSize(); col++) {
            float delta = m_diffusivity * (((gridCopy.getPlotValueAtPosition(row+1, col) - 2*gridCopy.getPlotValueAtPosition(row, col) + gridCopy.getPlotValueAtPosition(row-1, col)) / powf(gridCopy.getDeltaX(), 2.0f)) + 
                                 ((gridCopy.getPlotValueAtPosition(row, col+1) - 2*gridCopy.getPlotValueAtPosition(row, col) + gridCopy.getPlotValueAtPosition(row, col-1)) / powf(gridCopy.getDeltaY(), 2.0f))) * deltaTime;
            m_grid.setPlotValueAtPosition(row, col, gridCopy.getPlotValueAtPosition(row, col) + delta);
        }
    }
}

void HeatEquation::simulateStepEulerImplicit(const float& deltaTime) {
    const int matrixDimension = m_grid.getGridRowSize() * m_grid.getGridColSize();

    // PCG Solvers and the out value containers
    SparsePCGSolver<float> sparsePCGSolver;
    std::vector<float> result(matrixDimension);
    float residual;
    int iterations;

    // Setting up the matrix
    SparseMatrix<float> matrix(matrixDimension, 5);
    const float lambdaX = m_diffusivity * deltaTime / powf(m_grid.getDeltaX(), 2.0f);
    const float lambdaY = m_diffusivity * deltaTime / powf(m_grid.getDeltaY(), 2.0f);
    for (unsigned int matrixRow = 0; matrixRow < matrixDimension; matrixRow++) {
        int gridRow = matrixRow / m_grid.getGridColSize();
        int gridCol = matrixRow % m_grid.getGridColSize();

        int matrixCol_ImJ = (gridRow-1) * m_grid.getGridColSize() + gridCol;
        int matrixCol_IJm = gridRow * m_grid.getGridColSize() + (gridCol-1);
        int matrixCol_IJ = matrixRow;
        int matrixCol_IJp = gridRow * m_grid.getGridColSize() + (gridCol+1);
        int matrixCol_IpJ = (gridRow+1) * m_grid.getGridColSize() + gridCol;

        if ((0 <= matrixCol_ImJ) && (matrixCol_ImJ < matrixDimension)) {
            matrix.set_element(matrixRow, matrixCol_ImJ, -lambdaX);
        }
        if ((0 <= matrixCol_IJm) && (matrixCol_IJm < matrixDimension)) {
            matrix.set_element(matrixRow, matrixCol_IJm, -lambdaY);
        }

        matrix.set_element(matrixRow, matrixCol_IJ, 1 + 2*lambdaX + 2*lambdaY);

        if ((0 <= matrixCol_IJp) && (matrixCol_IJp < matrixDimension)) {
            matrix.set_element(matrixRow, matrixCol_IJp, -lambdaY);
        }
        if ((0 <= matrixCol_IpJ) && (matrixCol_IpJ < matrixDimension)) {
            matrix.set_element(matrixRow, matrixCol_IpJ, -lambdaX);
        }
    }

    // Setting up the rhs
    std::vector<float> rhs(matrixDimension);
    for (unsigned int row = 0; row < m_grid.getGridRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getGridColSize(); col++) {
            rhs[row * m_grid.getGridColSize() + col] = m_grid.getPlotValueAtPosition(row, col);
        }
    }

    // Solve
    sparsePCGSolver.solve(matrix, rhs, result, residual, iterations);

    // Write result to grid
    for (unsigned int row = 0; row < m_grid.getGridRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getGridColSize(); col++) {
            m_grid.setPlotValueAtPosition(row, col, result[row * m_grid.getGridColSize() + col]);
        }
    }
}

HeatEquation::HeatEquation(Grid &grid, const float& diffusivity, const IntegrationMode &integrationMode): m_grid{grid}, m_diffusivity{diffusivity}, m_integrationMode{integrationMode} {}

void HeatEquation::setDiffusivity(const float &diffusivity) {
    m_diffusivity = diffusivity;
}

void HeatEquation::simulateStep(const float& deltaTime) {
    switch (m_integrationMode) {
    case HeatEquation::IntegrationMode::EULER_EXPLICIT:
        simulateStepEulerExplicit(deltaTime);
        break;
    case HeatEquation::IntegrationMode::EULER_IMPLICIT:
        simulateStepEulerImplicit(deltaTime);
        break;
    default:
        break;
    }
    m_grid.setTime(m_grid.getTime() + deltaTime);
}