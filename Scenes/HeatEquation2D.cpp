#include "HeatEquation2D.hpp"
#include <cmath>

float HeatEquation2D::getDiscretizationAtPosition(const int& row, const int& col) const {
    if ((row < 0) || (row >= m_discretizationRowSize) || (col < 0) || (col >= m_discretizationColSize)) 
        return 0.0f;
    return m_discretization[row * m_discretizationColSize + col];
}

void HeatEquation2D::setDiscretizationAtPosition(const int& row, const int& col, const float& value) {
    if ((row < 0) || (row >= m_discretizationRowSize) || (col < 0) || (col >= m_discretizationColSize)) 
        return;
    m_discretization[row * m_discretizationColSize + col] = value;
}

HeatEquation2D::HeatEquation2D(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, 
                               const float& v, const unsigned int& discretizationRowSize, const unsigned int& discretizationColSize, float*const& initialDiscretization): 
    m_xBoundaryMin{xBoundaryMin}, m_xBoundaryMax{xBoundaryMax}, m_yBoundaryMin{yBoundaryMin}, m_yBoundaryMax{yBoundaryMax},
    m_time{0.0f}, m_v{v}, m_discretizationRowSize{discretizationRowSize}, m_discretizationColSize{discretizationColSize} {
    // Creating the state 2D array
    m_discretization = std::make_unique<float[]>(m_discretizationRowSize * m_discretizationColSize);
    for (unsigned int i = 0; i < m_discretizationRowSize * m_discretizationColSize; i++) {
        m_discretization[i] = initialDiscretization[i];
    }
}

// float HeatEquation2D::getValue(const float &x, const float &y) const {
//     // TODO: Add Interpolation for getValue
//     if ((x < m_xBoundaryMin) || (x > m_xBoundaryMax) ||
//         (y < m_yBoundaryMin) || (y > m_yBoundaryMax)) return 0.0f;
    
//     float col = (x - m_xBoundaryMin) / m_discretizationColSize;
//     float row = (y - m_yBoundaryMin) / m_discretizationRowSize;
//     return 0.0f;
// }


void HeatEquation2D::simulateStep(float deltaTime) {
    float deltaX = (m_xBoundaryMax - m_xBoundaryMin) / (m_discretizationColSize-1);
    float deltaY = (m_yBoundaryMax - m_yBoundaryMin) / (m_discretizationRowSize-1);
    for (unsigned int row = 0; row < m_discretizationRowSize; row++) {
        for (unsigned int col = 0; col < m_discretizationColSize; col++) {
            float delta = m_v * (((getDiscretizationAtPosition(row+1, col) - 2*getDiscretizationAtPosition(row, col) + getDiscretizationAtPosition(row-1, col)) / powf(deltaY, 2.0f)) + 
                                 ((getDiscretizationAtPosition(row, col+1) - 2*getDiscretizationAtPosition(row, col) + getDiscretizationAtPosition(row, col-1)) / powf(deltaX, 2.0f))) * deltaTime;
            setDiscretizationAtPosition(row, col, getDiscretizationAtPosition(row, col) + delta);
        }
    }
}
