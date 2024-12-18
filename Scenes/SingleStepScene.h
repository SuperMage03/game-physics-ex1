#include "Scene.h"
#include "HeatEquation.h"
#include "Grid.h"

class SingleStepScene : public Scene {
private:
    const float initialState[18] = { 6.0f,  5.0f,  1.0f, -1.0f, -2.0f, -1.0f,
                                     4.0f,  3.0f,  0.0f, -1.0f, -3.0f, -1.0f,
                                     3.0f,  2.0f, -1.0f, -2.0f, -4.0f, -2.0f};
    Grid grid{0.0f, 2.0f, 0.0f, 4.0f, 3, 6, initialState};
    HeatEquation HeatEquation{grid, 0.1f};    
public:
    void init() override;
};