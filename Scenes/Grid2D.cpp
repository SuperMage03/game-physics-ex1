#include "Grid2D.hpp"
#include <random>

float Grid2D::getXBoundaryMin() const {
    return m_xBoundaryMin;
}

float Grid2D::getXBoundaryMax() const {
    return m_xBoundaryMax;
}

float Grid2D::getYBoundaryMin() const {
    return m_yBoundaryMin;
}

float Grid2D::getYBoundaryMax() const {
    return m_yBoundaryMax;
}

float Grid2D::getGridRowSize() const {
    return m_gridRowSize;
}

float Grid2D::getGridColSize() const {
    return m_gridColSize;
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
    m_gridRowSize{copy.m_gridRowSize}, m_gridColSize{copy.m_gridColSize},
    m_deltaX{copy.m_deltaX}, m_deltaY{copy.m_deltaY}, m_time{copy.m_time} {
    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        m_plotValues[i] = copy.m_plotValues[i];
    }
}

Grid2D::Grid2D(const float &xBoundaryMin, const float &xBoundaryMax, const float &yBoundaryMin, const float &yBoundaryMax,
               const unsigned int &discretizationRowSize, const unsigned int &discretizationColSize, const float *const &initialDiscretization): 
               m_xBoundaryMin{xBoundaryMin}, m_xBoundaryMax{xBoundaryMax}, m_yBoundaryMin{yBoundaryMin}, m_yBoundaryMax{yBoundaryMax},
               m_deltaX{(xBoundaryMax - xBoundaryMin) / (discretizationRowSize + 1)},
               m_deltaY{(yBoundaryMax - yBoundaryMin) / (discretizationColSize + 1)},
               m_time{0.0f}, m_gridRowSize{discretizationRowSize}, m_gridColSize{discretizationColSize} {
    // Creating the state 2D array
    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        m_plotValues[i] = initialDiscretization[i];
    }
}

float Grid2D::getPlotValueAtPosition(const int& row, const int& col) const {
    if ((row < 0) || (row >= m_gridRowSize) || (col < 0) || (col >= m_gridColSize)) 
        return 0.0f;
    return m_plotValues[row * m_gridColSize + col];
}

void Grid2D::setPlotValueAtPosition(const int& row, const int& col, const float& value) {
    if ((row < 0) || (row >= m_gridRowSize) || (col < 0) || (col >= m_gridColSize)) 
        return;
    m_plotValues[row * m_gridColSize + col] = value;
}

void Grid2D::randomizePlotValues() {
    // Initialize random number generator
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, 0.5f);

    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        // Generate Gaussian noise
        float noise = distribution(generator);
        m_plotValues[i] = noise;
    }
}

void Grid2D::drawGrid(Renderer& renderer, const glm::vec3& origin, const float& scale) const {
    const float center_row_idx = (m_gridRowSize-1) / 2.0f;
    const float center_col_idx = (m_gridColSize-1) / 2.0f;

    for (int row = -1; row < static_cast<int>(m_gridRowSize)+1; row++) {
        for (int col = -1; col < static_cast<int>(m_gridColSize)+1; col++) {
            renderer.drawSphere(scale * (origin + glm::vec3((row-center_row_idx) * m_deltaX, (col-center_col_idx) * m_deltaY, getPlotValueAtPosition(row, col))), 0.05f);
        }
    }
    for (int row = -1; row < static_cast<int>(m_gridRowSize)+1; row++) {
        for (int col = 0; col < static_cast<int>(m_gridColSize)+1; col++) {
            renderer.drawLine(scale * (origin + glm::vec3((row-center_row_idx) * m_deltaX, (col-center_col_idx) * m_deltaY, getPlotValueAtPosition(row, col))), scale * (origin + glm::vec3((row-center_row_idx) * m_deltaX, (col-center_col_idx-1) * m_deltaY, getPlotValueAtPosition(row, col-1))), glm::vec4(1.0f));
        }
    }
    for (int row = 0; row < static_cast<int>(m_gridRowSize)+1; row++) {
        for (int col = -1; col < static_cast<int>(m_gridColSize)+1; col++) {
            renderer.drawLine(scale * (origin + glm::vec3((row-center_row_idx) * m_deltaX, (col-center_col_idx) * m_deltaY, getPlotValueAtPosition(row, col))), scale * (origin + glm::vec3((row-center_row_idx-1) * m_deltaX, (col-center_col_idx) * m_deltaY, getPlotValueAtPosition(row-1, col))), glm::vec4(1.0f));
        }
    }
}

// float Grid2D::getValue(const float &x, const float &y) const {
//     // TODO: Add Interpolation for getValue
//     if ((x < m_xBoundaryMin) || (x > m_xBoundaryMax) ||
//         (y < m_yBoundaryMin) || (y > m_yBoundaryMax)) return 0.0f;
    
//     float col = (x - m_xBoundaryMin) / m_gridColSize;
//     float row = (y - m_yBoundaryMin) / m_gridRowSize;
//     return 0.0f;
// }
