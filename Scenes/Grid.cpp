#include "Grid.h"
#include <random>

float Grid::getXBoundaryMin() const {
    return m_xBoundaryMin;
}

float Grid::getXBoundaryMax() const {
    return m_xBoundaryMax;
}

float Grid::getYBoundaryMin() const {
    return m_yBoundaryMin;
}

float Grid::getYBoundaryMax() const {
    return m_yBoundaryMax;
}

unsigned int Grid::getGridRowSize() const {
    return m_gridRowSize;
}

unsigned int Grid::getGridColSize() const {
    return m_gridColSize;
}

float Grid::getDeltaX() const {
    return m_deltaX;
}

float Grid::getDeltaY() const {
    return m_deltaY;
}

float Grid::getTime() const {
    return m_time;
}

void Grid::setTime(const float& time) {
    m_time = time;
}

Grid::Grid(const Grid &copy): 
    m_xBoundaryMin{copy.m_xBoundaryMin}, m_xBoundaryMax{copy.m_xBoundaryMax}, m_yBoundaryMin{copy.m_yBoundaryMin}, m_yBoundaryMax{copy.m_yBoundaryMax},
    m_gridRowSize{copy.m_gridRowSize}, m_gridColSize{copy.m_gridColSize},
    m_deltaX{copy.m_deltaX}, m_deltaY{copy.m_deltaY}, m_time{copy.m_time} {
    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        m_plotValues[i] = copy.m_plotValues[i];
    }
}

Grid::Grid(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, 
               const unsigned int& gridRowSize, const unsigned int& gridColSize, const float*const& initialPlotValues): 
               m_xBoundaryMin{xBoundaryMin}, m_xBoundaryMax{xBoundaryMax}, m_yBoundaryMin{yBoundaryMin}, m_yBoundaryMax{yBoundaryMax},
               m_time{0.0f}, m_gridRowSize{gridRowSize}, m_gridColSize{gridColSize} {
    calculateDirectionalDelta();
    // Creating the state 2D array
    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        m_plotValues[i] = initialPlotValues[i];
    }
}

float Grid::getPlotValueAtPosition(const int& row, const int& col) const {
    if ((row < 0) || (row >= m_gridRowSize) || (col < 0) || (col >= m_gridColSize)) 
        return 0.0f;
    return m_plotValues[row * m_gridColSize + col];
}

void Grid::setPlotValueAtPosition(const int& row, const int& col, const float& value) {
    if ((row < 0) || (row >= m_gridRowSize) || (col < 0) || (col >= m_gridColSize)) 
        return;
    m_plotValues[row * m_gridColSize + col] = value;
}

void Grid::calculateDirectionalDelta() {
    m_deltaX = (m_xBoundaryMax - m_xBoundaryMin) / (m_gridRowSize + 1);
    m_deltaY = (m_yBoundaryMax - m_yBoundaryMin) / (m_gridColSize + 1);
}

void Grid::resizeDomain(const float &xBoundaryMin, const float &xBoundaryMax, const float &yBoundaryMin, const float &yBoundaryMax) {
    m_xBoundaryMin = xBoundaryMin;
    m_xBoundaryMax = xBoundaryMax;
    m_yBoundaryMin = yBoundaryMin;
    m_yBoundaryMax = yBoundaryMax;
    calculateDirectionalDelta();

    gaussianRandomizePlotValues();
}

void Grid::resizeGrid(const unsigned int &rowSize, const unsigned int &colSize) {
    m_gridRowSize = rowSize;
    m_gridColSize = colSize;
    calculateDirectionalDelta();

    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    gaussianRandomizePlotValues();
}

void Grid::gaussianRandomizePlotValues() {
    const float mean = 0.0f;
    const float standardDeviation = 0.2f * (((m_xBoundaryMax-m_xBoundaryMin)+(m_yBoundaryMax-m_yBoundaryMin)) / 2.0f);
    // Initialize random number generator
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(mean, standardDeviation);

    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        // Generate Gaussian noise
        float noise = distribution(generator);
        m_plotValues[i] = noise;
    }
}

void Grid::drawGrid(Renderer& renderer, const glm::vec3& origin, const float& scale) const {
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