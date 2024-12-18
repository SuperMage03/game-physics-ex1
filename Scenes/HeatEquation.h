#pragma once
#include "Grid.h"

class HeatEquation {
public:
    enum class IntegrationMode {
        EULER_EXPLICIT,
        EULER_IMPLICIT,
    };
private:
    Grid& m_grid;
    float m_diffusivity;
    IntegrationMode m_integrationMode;
    void simulateStepEulerExplicit(const float& deltaTime);
    void simulateStepEulerImplicit(const float& deltaTime);
public:
    HeatEquation(Grid& grid, const float& diffusivity, const IntegrationMode& integrationMode=IntegrationMode::EULER_EXPLICIT);
    void setDiffusivity(const float& diffusivity);
    void simulateStep(const float& deltaTime);
};