#include "Scene.h"
#include "HeatEquation2D.hpp"
#include "Grid2D.hpp"

class Scene4 : public Scene {
private:
    float m_xBoundaryMin = 0.0f;
    float m_xBoundaryMax = 1.0f;
    float m_yBoundaryMin = 0.0f;
    float m_yBoundaryMax = 1.0f;

    int m_rowSize = 16;
    int m_colSize = 16;

    bool m_isPaused = true;
    float m_step = 0.01f;
    float m_diffusivity = 0.1f;

    float initialState[18] = { 6.0f,  5.0f,  1.0f, -1.0f, -2.0f, -1.0f,
                               4.0f,  3.0f,  0.0f, -1.0f, -3.0f, -1.0f,
                               3.0f,  2.0f, -1.0f, -2.0f, -4.0f, -2.0f};

    Grid2D grid{m_xBoundaryMin, m_xBoundaryMax, m_yBoundaryMin, m_yBoundaryMax, static_cast<unsigned int>(m_rowSize), static_cast<unsigned int>(m_colSize), initialState};
    HeatEquation2D heatEquation2D{grid, m_diffusivity, HeatEquation2D::IntegrationMode::EULER_IMPLICIT};
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer& renderer) override;
    void onGUI() override;
};
