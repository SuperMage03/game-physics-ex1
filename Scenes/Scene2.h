#include "Scene.h"
#include "HeatEquation2D.hpp"
#include "Grid2D.hpp"

class Scene2 : public Scene {
private:
    float initialState[18] = { 6.0f,  5.0f,  1.0f, -1.0f, -2.0f, -1.0f,
                               4.0f,  3.0f,  0.0f, -1.0f, -3.0f, -1.0f,
                               3.0f,  2.0f, -1.0f, -2.0f, -4.0f, -2.0f};

    Grid2D grid{0.0f, 10.0f, 0.0f, 10.0f, 16, 16, initialState};
    HeatEquation2D heatEquation2D{grid, 0.1f};
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer& renderer) override;
};
