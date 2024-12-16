#include "Scene1.h"
#include <iostream>

void Scene1::init() {
    heatEquation2D.simulateStep(0.1f);
    std::cout << heatEquation2D.getDiscretizationAtPosition(1, 3) << std::endl;
    std::cout << heatEquation2D.getDiscretizationAtPosition(0, 3) << std::endl;
    std::cout << heatEquation2D.getDiscretizationAtPosition(0, 5) << std::endl;
}
