#pragma once
#include "Grid2D.hpp"

class HeatEquation2D {
public:
    enum class IntegrationMode {
        EULER_EXPLICIT,
        EULER_IMPLICIT,
    };
private:
    Grid2D& m_grid;
    float m_v;
    IntegrationMode m_integrationMode;
    void simulateStepEulerExplicit(float deltaTime);
public:
    HeatEquation2D(Grid2D& grid, const float& v, const IntegrationMode& integrationMode=IntegrationMode::EULER_EXPLICIT);
    void simulateStep(float deltaTime);
};
