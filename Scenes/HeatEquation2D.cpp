#include "HeatEquation2D.hpp"
#include <cmath>

float HeatEquation2D::getDiscretization(const float *const &discretization, const int &rowSize, const int &colSize, const int &row, const int &col) {
    if ((row < 0) || (row >= rowSize) || (col < 0) || (col >= colSize)) 
        return 0.0f;
    return discretization[row * colSize + col];
}

void HeatEquation2D::setDiscretization(float *const &discretization, const int &rowSize, const int &colSize, const int &row, const int &col, const float &value) {
    if ((row < 0) || (row >= rowSize) || (col < 0) || (col >= colSize)) 
        return;
    discretization[row * colSize + col] = value;
}

HeatEquation2D::HeatEquation2D(const float &xBoundaryMin, const float &xBoundaryMax, const float &yBoundaryMin, const float &yBoundaryMax,
                               const float &v, const unsigned int &discretizationRowSize, const unsigned int &discretizationColSize, float *const &initialDiscretization) : m_xBoundaryMin{xBoundaryMin}, m_xBoundaryMax{xBoundaryMax}, m_yBoundaryMin{yBoundaryMin}, m_yBoundaryMax{yBoundaryMax},
                                                                                                                                                                            m_time{0.0f}, m_v{v}, m_discretizationRowSize{discretizationRowSize}, m_discretizationColSize{discretizationColSize} {
    // Creating the state 2D array
    m_discretization = std::make_unique<float[]>(m_discretizationRowSize * m_discretizationColSize);
    for (unsigned int i = 0; i < m_discretizationRowSize * m_discretizationColSize; i++) {
        m_discretization[i] = initialDiscretization[i];
    }
}

float HeatEquation2D::getDiscretizationAtPosition(const int& row, const int& col) const {
    return getDiscretization(m_discretization.get(), m_discretizationRowSize, m_discretizationColSize, row, col);
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
    auto discretizationCopy = std::make_unique<float[]>(m_discretizationRowSize * m_discretizationColSize);
    for (unsigned int i = 0; i < m_discretizationRowSize * m_discretizationColSize; i++) {
        discretizationCopy[i] = m_discretization[i];
    }

    float deltaX = (m_xBoundaryMax - m_xBoundaryMin) / (m_discretizationColSize+1);
    float deltaY = (m_yBoundaryMax - m_yBoundaryMin) / (m_discretizationRowSize+1);

    for (unsigned int row = 0; row < m_discretizationRowSize; row++) {
        for (unsigned int col = 0; col < m_discretizationColSize; col++) {
            float delta = m_v * (((getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row+1, col) - 2*getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row, col) + getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row-1, col)) / powf(deltaY, 2.0f)) + 
                                 ((getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row, col+1) - 2*getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row, col) + getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row, col-1)) / powf(deltaX, 2.0f))) * deltaTime;
            setDiscretization(m_discretization.get(), m_discretizationRowSize, m_discretizationColSize, row, col, getDiscretization(discretizationCopy.get(), m_discretizationRowSize, m_discretizationColSize, row, col) + delta);
        }
    }
    m_time += deltaTime;
}
