#include "Grid2D.hpp"
#include <random>

glm::vec2 Grid2D::gridToCoordinate(const int &row, const int &col) const {
    // +1 for the fact that row and col doesn't count the boundary points whilst the deltaX and deltaY do
    return glm::vec2((row+1) * m_deltaX, (col+1) * m_deltaY);
}

void Grid2D::calculateDirectionalDelta() {
    m_deltaX = (m_xBoundaryMax - m_xBoundaryMin) / (m_gridRowSize + 1);
    m_deltaY = (m_yBoundaryMax - m_yBoundaryMin) / (m_gridColSize + 1);
}

int Grid2D::flattenFunction(const int &row, const int &col) const {
    return row * m_gridColSize + col;
}

glm::vec2 Grid2D::unflattenFunction(const int &flattenedValue) const {
    return glm::vec2(flattenedValue / getGridColSize(), flattenedValue % getGridColSize());
}

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

unsigned int Grid2D::getGridRowSize() const {
    return m_gridRowSize;
}

unsigned int Grid2D::getGridColSize() const {
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
    m_xBoundaryCondition{copy.m_xBoundaryCondition}, m_yBoundaryCondition{copy.m_yBoundaryCondition},
    m_gridRowSize{copy.m_gridRowSize}, m_gridColSize{copy.m_gridColSize},
    m_deltaX{copy.m_deltaX}, m_deltaY{copy.m_deltaY}, m_time{copy.m_time} {
    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        m_plotValues[i] = copy.m_plotValues[i];
    }
}

Grid2D::Grid2D(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, 
               const unsigned int& gridRowSize, const unsigned int& gridColSize, 
               float (*const&xBoundaryCondition)(const float& x, const float& t), float (*const&yBoundaryCondition)(const float& y, const float& t), const float*const& initialPlotValues): 
               m_xBoundaryMin{xBoundaryMin}, m_xBoundaryMax{xBoundaryMax}, m_yBoundaryMin{yBoundaryMin}, m_yBoundaryMax{yBoundaryMax},
               m_xBoundaryCondition{xBoundaryCondition}, m_yBoundaryCondition{yBoundaryCondition}, m_time{0.0f}, m_gridRowSize{gridRowSize}, m_gridColSize{gridColSize} {
    calculateDirectionalDelta();
    // Creating the state 2D array
    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    for (unsigned int i = 0; i < m_gridRowSize * m_gridColSize; i++) {
        m_plotValues[i] = initialPlotValues[i];
    }
}

float Grid2D::getPlotValueAtPosition(const int& row, const int& col) const {
    glm::vec2 coordinationPosition = gridToCoordinate(row, col);
    if ((coordinationPosition.x <= m_xBoundaryMin) || (coordinationPosition.x >= m_xBoundaryMax)) 
        return m_xBoundaryCondition(coordinationPosition.x, m_time);
    if ((coordinationPosition.y <= m_yBoundaryMin) || (coordinationPosition.y >= m_yBoundaryMax))
        return m_yBoundaryCondition(coordinationPosition.y, m_time);
    return m_plotValues[flattenFunction(row, col)];
}

void Grid2D::setPlotValueAtPosition(const int& row, const int& col, const float& value) {
    if ((row < 0) || (row >= m_gridRowSize) || (col < 0) || (col >= m_gridColSize)) 
        return;
    m_plotValues[flattenFunction(row, col)] = value;
}

void Grid2D::resizeDomain(const float &xBoundaryMin, const float &xBoundaryMax, const float &yBoundaryMin, const float &yBoundaryMax) {
    m_xBoundaryMin = xBoundaryMin;
    m_xBoundaryMax = xBoundaryMax;
    m_yBoundaryMin = yBoundaryMin;
    m_yBoundaryMax = yBoundaryMax;
    calculateDirectionalDelta();

    gaussianRandomizePlotValues();
}

void Grid2D::resizeGrid(const unsigned int &rowSize, const unsigned int &colSize) {
    m_gridRowSize = rowSize;
    m_gridColSize = colSize;
    calculateDirectionalDelta();

    m_plotValues = std::make_unique<float[]>(m_gridRowSize * m_gridColSize);
    gaussianRandomizePlotValues();
}

void Grid2D::gaussianRandomizePlotValues() {
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
