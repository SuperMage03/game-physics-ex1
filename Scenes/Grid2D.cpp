#include "Grid2D.hpp"

float Grid2D::getDiscretizationRowSize() const {
    return m_discretizationRowSize;
}

float Grid2D::getDiscretizationColSize() const {
    return m_discretizationColSize;
}

float Grid2D::getDeltaX() const {
    return m_deltaX;
}

float Grid2D::getDeltaY() const {
    return m_deltaY;
}

float Grid2D::getTime() const {
    return m_time;
}

void Grid2D::setTime(const float& time) {
    m_time = time;
}

Grid2D::Grid2D(const Grid2D &copy): 
    m_xBoundaryMin{copy.m_xBoundaryMin}, m_xBoundaryMax{copy.m_xBoundaryMax}, m_yBoundaryMin{copy.m_yBoundaryMin}, m_yBoundaryMax{copy.m_yBoundaryMax},
    m_discretizationRowSize{copy.m_discretizationRowSize}, m_discretizationColSize{copy.m_discretizationColSize},
    m_deltaX{copy.m_deltaX}, m_deltaY{copy.m_deltaY}, m_time{copy.m_time} {
    m_discretization = std::make_unique<float[]>(m_discretizationRowSize * m_discretizationColSize);
    for (unsigned int i = 0; i < m_discretizationRowSize * m_discretizationColSize; i++) {
        m_discretization[i] = copy.m_discretization[i];
    }
}

Grid2D::Grid2D(const float &xBoundaryMin, const float &xBoundaryMax, const float &yBoundaryMin, const float &yBoundaryMax,
               const unsigned int &discretizationRowSize, const unsigned int &discretizationColSize, float *const &initialDiscretization): 
               m_xBoundaryMin{xBoundaryMin}, m_xBoundaryMax{xBoundaryMax}, m_yBoundaryMin{yBoundaryMin}, m_yBoundaryMax{yBoundaryMax},
               m_deltaX{(xBoundaryMax - xBoundaryMin) / (discretizationRowSize + 1)},
               m_deltaY{(yBoundaryMax - yBoundaryMin) / (discretizationColSize + 1)},
               m_time{0.0f}, m_discretizationRowSize{discretizationRowSize}, m_discretizationColSize{discretizationColSize} {
    // Creating the state 2D array
    m_discretization = std::make_unique<float[]>(m_discretizationRowSize * m_discretizationColSize);
    for (unsigned int i = 0; i < m_discretizationRowSize * m_discretizationColSize; i++) {
        m_discretization[i] = initialDiscretization[i];
    }
}

float Grid2D::getDiscretizationAtPosition(const int& row, const int& col) const {
    if ((row < 0) || (row >= m_discretizationRowSize) || (col < 0) || (col >= m_discretizationColSize)) 
        return 0.0f;
    return m_discretization[row * m_discretizationColSize + col];
}

void Grid2D::setDiscretizationAtPosition(const int& row, const int& col, const float& value) {
    if ((row < 0) || (row >= m_discretizationRowSize) || (col < 0) || (col >= m_discretizationColSize)) 
        return;
    m_discretization[row * m_discretizationColSize + col] = value;
}


void Grid2D::drawGrid(Renderer &renderer) const {
    for (int row = -1; row < static_cast<int>(m_discretizationRowSize)+1; row++) {
        for (int col = -1; col < static_cast<int>(m_discretizationColSize)+1; col++) {
            renderer.drawSphere(glm::vec3(row * m_deltaX, col * m_deltaY, getDiscretizationAtPosition(row, col)), 0.1f);
        }
    }
    for (int row = -1; row < static_cast<int>(m_discretizationRowSize)+1; row++) {
        for (int col = 0; col < static_cast<int>(m_discretizationColSize)+1; col++) {
            renderer.drawLine(glm::vec3(row * m_deltaX, col * m_deltaY, getDiscretizationAtPosition(row, col)), glm::vec3(row * m_deltaX, (col-1) * m_deltaY, getDiscretizationAtPosition(row, col-1)), glm::vec4(1.0f));
        }
    }
    for (int row = 0; row < static_cast<int>(m_discretizationRowSize)+1; row++) {
        for (int col = -1; col < static_cast<int>(m_discretizationColSize)+1; col++) {
            renderer.drawLine(glm::vec3(row * m_deltaX, col * m_deltaY, getDiscretizationAtPosition(row, col)), glm::vec3((row-1) * m_deltaX, col * m_deltaY, getDiscretizationAtPosition(row-1, col)), glm::vec4(1.0f));
        }
    }
}

// float Grid2D::getValue(const float &x, const float &y) const {
//     // TODO: Add Interpolation for getValue
//     if ((x < m_xBoundaryMin) || (x > m_xBoundaryMax) ||
//         (y < m_yBoundaryMin) || (y > m_yBoundaryMax)) return 0.0f;
    
//     float col = (x - m_xBoundaryMin) / m_discretizationColSize;
//     float row = (y - m_yBoundaryMin) / m_discretizationRowSize;
//     return 0.0f;
// }
