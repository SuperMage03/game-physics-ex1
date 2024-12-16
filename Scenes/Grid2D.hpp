#pragma once
#include <memory>
#include <Renderer.h>

class Grid2D {
private:
    const float m_xBoundaryMin;
    const float m_xBoundaryMax;
    const float m_yBoundaryMin;
    const float m_yBoundaryMax;
    const unsigned int m_discretizationRowSize;
    const unsigned int m_discretizationColSize;
    const float m_deltaX;
    const float m_deltaY;

    float m_time;
    std::unique_ptr<float[]> m_discretization;
public:
    float getDiscretizationRowSize() const;
    float getDiscretizationColSize() const;

    float getDeltaX() const;
    float getDeltaY() const;

    float getTime() const;
    void setTime(const float& time);

    Grid2D(const Grid2D& copy);
    Grid2D(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, const unsigned int& discretizationRowSize, const unsigned int& discretizationColSize, float*const& initialDiscretization);

    float getDiscretizationAtPosition(const int& row, const int& col) const;
    void setDiscretizationAtPosition(const int& row, const int& col, const float& value);

    // float getValue(const float& x, const float& y) const;
    void drawGrid(Renderer& renderer) const;
};

