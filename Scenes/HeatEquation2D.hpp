#pragma once
#include <memory>

class HeatEquation2D {
private:
    float m_xBoundaryMin;
    float m_xBoundaryMax;
    float m_yBoundaryMin;
    float m_yBoundaryMax;
    float m_time;
    float m_v;
    unsigned int m_discretizationRowSize;
    unsigned int m_discretizationColSize;
    std::unique_ptr<float[]> m_discretization;
    void setDiscretizationAtPosition(const int& row, const int& col, const float& value);
public:
    HeatEquation2D(const float& xBoundaryMin, const float& xBoundaryMax, const float& yBoundaryMin, const float& yBoundaryMax, const float& v, const unsigned int& discretizationRowSize, const unsigned int& discretizationColSize, float*const& initialDiscretization);
    float getDiscretizationAtPosition(const int& row, const int& col) const;
    // float getValue(const float& x, const float& y) const;
    void simulateStep(float deltaTime);
};
