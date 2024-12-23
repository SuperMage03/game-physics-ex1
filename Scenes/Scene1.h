#include "Scene.h"
#include "HeatEquation2D.hpp"
#include "Grid2D.hpp"

class Scene1 : public Scene {
private:
    const float initialState[18] = { 6.0f,  5.0f,  1.0f, -1.0f, -2.0f, -1.0f,
                                     4.0f,  3.0f,  0.0f, -1.0f, -3.0f, -1.0f,
                                     3.0f,  2.0f, -1.0f, -2.0f, -4.0f, -2.0f};
    Grid2D grid{0.0f, 2.0f, 0.0f, 4.0f, 3, 6, [](const float& x, const float& t)->float {return 0.0f;}, [](const float& y, const float& t)->float {return 0.0f;}, initialState};
    HeatEquation2D heatEquation2D{grid, 0.1f};    
public:
    void init() override;
};
