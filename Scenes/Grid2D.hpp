#pragma once
#include <memory>
#include <Renderer.h>
#include <glm/glm.hpp>

class Grid2D {
private:
    const float m_xBoundaryMin;
    const float m_xBoundaryMax;
    const float m_yBoundaryMin;
    const float m_yBoundaryMax;
    const unsigned int m_gridRowSize;
    const unsigned int m_gridColSize;
    const float m_deltaX;
    const float m_deltaY;

    float m_time;
    std::unique_ptr<float[]> m_plotValues;
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

    Grid2D(const Grid2D& copy);
    Grid2D(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, const unsigned int& discretizationRowSize, const unsigned int& discretizationColSize, const float*const& initialDiscretization);

    float getPlotValueAtPosition(const int& row, const int& col) const;
    void setPlotValueAtPosition(const int& row, const int& col, const float& value);

    void randomizePlotValues();

    // float getValue(const float& x, const float& y) const;
    void drawGrid(Renderer& renderer, const glm::vec3& origin = glm::vec3(0.0f), const float& scale = 1.0f) const;
};
