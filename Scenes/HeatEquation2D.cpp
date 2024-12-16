#include "HeatEquation2D.hpp"
#include <cmath>

void HeatEquation2D::simulateStepEulerExplicit(float deltaTime) {
    Grid2D gridCopy{m_grid};

    for (unsigned int row = 0; row < m_grid.getDiscretizationRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getDiscretizationColSize(); col++) {
            float delta = m_v * (((gridCopy.getDiscretizationAtPosition(row+1, col) - 2*gridCopy.getDiscretizationAtPosition(row, col) + gridCopy.getDiscretizationAtPosition(row-1, col)) / powf(gridCopy.getDeltaX(), 2.0f)) + 
                                 ((gridCopy.getDiscretizationAtPosition(row, col+1) - 2*gridCopy.getDiscretizationAtPosition(row, col) + gridCopy.getDiscretizationAtPosition(row, col-1)) / powf(gridCopy.getDeltaY(), 2.0f))) * deltaTime;
            m_grid.setDiscretizationAtPosition(row, col, gridCopy.getDiscretizationAtPosition(row, col) + delta);
        }
    }
}

HeatEquation2D::HeatEquation2D(Grid2D &grid, const float& v, const IntegrationMode &integrationMode) : m_grid{grid}, m_v{v}, m_integrationMode{integrationMode} {}

void HeatEquation2D::simulateStep(float deltaTime) {
    switch (m_integrationMode) {
    case HeatEquation2D::IntegrationMode::EULER_EXPLICIT:
        simulateStepEulerExplicit(deltaTime);
        break;
    
    default:
        break;
    }
    m_grid.setTime(m_grid.getTime() + deltaTime);
}
