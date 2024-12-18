#include "Scene.h"
#include "HeatEquation2D.hpp"
#include "Grid2D.hpp"

class Scene3 : public Scene {
private:
    bool m_isPaused = true;
    float m_step = 0.01f;
    float m_diffusivity = 0.1f;

    float initialState[18] = { 6.0f,  5.0f,  1.0f, -1.0f, -2.0f, -1.0f,
                               4.0f,  3.0f,  0.0f, -1.0f, -3.0f, -1.0f,
                               3.0f,  2.0f, -1.0f, -2.0f, -4.0f, -2.0f};

    Grid2D grid{0.0f, 1.0f, 0.0f, 1.0f, 16, 16, initialState};
    HeatEquation2D heatEquation2D{grid, m_diffusivity, HeatEquation2D::IntegrationMode::EULER_IMPLICIT};
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer& renderer) override;
    void onGUI() override;
};
