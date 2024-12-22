#include "Scene.h"
#include <random>

class ImplicitSimulation: public Scene 
{
    virtual void init() override;
    virtual void onGUI() override;
    virtual void onDraw(Renderer &renderer) override;
    // void ExplicitSimulation::initializeExampleGrid();
    void ImplicitSimulation::initializeRandomNoise(glm::vec2 domainExtent, int mDegreesOfFreedom, int nDegreesOfFreedom);
    void ImplicitSimulation::explicitEulerSchemeStep();
    void ImplicitSimulation::implicitBTCSSchemeStep();

    struct TemperatureGrid {
        glm::vec2 domainExtent;
        int mDegreesOfFreedom;
        int nDegreesOfFreedom;
        float deltaX;
        float deltaY;
        std::vector<std::vector<float>> grid;
        float vThermalDiffusivity;
    };

    float deltaTTimeStepSize = 0.01f;
    TemperatureGrid tempGrid;

    // for random noise
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    public:
        ImplicitSimulation() : gen(rd()), dis(0.f, 1.f) {}
};