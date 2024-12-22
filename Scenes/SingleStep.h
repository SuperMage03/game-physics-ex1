#include "Scene.h"

class SingleStep : public Scene
{
    virtual void init() override;
    void SingleStep::initializeExampleGrid();
    void SingleStep::implicitEulerSchemeStep();

    struct TemperatureGrid {
        glm::vec2 domainExtent;
        int mDegreesOfFreedom;
        int nDegreesOfFreedom;
        float deltaX;
        float deltaY;
        std::vector<std::vector<float>> grid;
        float vThermalDiffusivity;
    };

    float deltaTTimeStepSize = 0.1f;
    TemperatureGrid tempGrid;
};