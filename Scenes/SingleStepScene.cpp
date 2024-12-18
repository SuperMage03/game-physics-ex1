#include "SingleStepScene.h"
#include <iostream>

void SingleStepScene::init() {
    HeatEquation.simulateStep(0.1f);
    std::cout << grid.getPlotValueAtPosition(1, 3) << std::endl;
    std::cout << grid.getPlotValueAtPosition(0, 3) << std::endl;
    std::cout << grid.getPlotValueAtPosition(0, 5) << std::endl;
}