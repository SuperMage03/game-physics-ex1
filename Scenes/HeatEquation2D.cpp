#include "HeatEquation2D.hpp"
#include <cmath>

void HeatEquation2D::simulateStepEulerExplicit(const float& deltaTime) {
    const Grid2D gridCopy{m_grid};

    for (unsigned int row = 0; row < m_grid.getGridRowSize(); row++) {
        for (unsigned int col = 0; col < m_grid.getGridColSize(); col++) {
            float delta = m_v * (((gridCopy.getPlotValueAtPosition(row+1, col) - 2*gridCopy.getPlotValueAtPosition(row, col) + gridCopy.getPlotValueAtPosition(row-1, col)) / powf(gridCopy.getDeltaX(), 2.0f)) + 
                                 ((gridCopy.getPlotValueAtPosition(row, col+1) - 2*gridCopy.getPlotValueAtPosition(row, col) + gridCopy.getPlotValueAtPosition(row, col-1)) / powf(gridCopy.getDeltaY(), 2.0f))) * deltaTime;
            m_grid.setPlotValueAtPosition(row, col, gridCopy.getPlotValueAtPosition(row, col) + delta);
        }
    }
}

void HeatEquation2D::simulateStepEulerImplicit(const float& deltaTime) {

}

HeatEquation2D::HeatEquation2D(Grid2D &grid, const float& v, const IntegrationMode &integrationMode): m_grid{grid}, m_v{v}, m_integrationMode{integrationMode} {}

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
