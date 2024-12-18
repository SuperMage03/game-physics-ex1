#pragma once
#include <memory>
#include <Renderer.h>
#include <glm/glm.hpp>

class Grid {
private:
    float m_xBoundaryMin;
    float m_xBoundaryMax;
    float m_yBoundaryMin;
    float m_yBoundaryMax;
    unsigned int m_gridRowSize;
    unsigned int m_gridColSize;
    
    float m_deltaX;
    float m_deltaY;

    float m_time;
    std::unique_ptr<float[]> m_plotValues;

    void calculateDirectionalDelta();
public:
    float getXBoundaryMin() const;
    float getXBoundaryMax() const;
    float getYBoundaryMin() const;
    float getYBoundaryMax() const;

    unsigned int getGridRowSize() const;
    unsigned int getGridColSize() const;

    float getDeltaX() const;
    float getDeltaY() const;

    float getTime() const;
    void setTime(const float& time);

    Grid(const Grid& copy);
    Grid(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, const unsigned int& gridRowSize, const unsigned int& gridColSize, const float*const& initialPlotValues);

    float getPlotValueAtPosition(const int& row, const int& col) const;
    void setPlotValueAtPosition(const int& row, const int& col, const float& value);

    void resizeDomain(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax);
    void resizeGrid(const unsigned int& rowSize, const unsigned int& colSize);

    void gaussianRandomizePlotValues();

    // float getValue(const float& x, const float& y) const;
    void drawGrid(Renderer& renderer, const glm::vec3& origin = glm::vec3(0.0f), const float& scale = 1.0f) const;
};