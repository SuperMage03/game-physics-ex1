#include "Scene1.h"
#include <iostream>

void Scene1::init() {
    heatEquation2D.simulateStep(0.1f);
    std::cout << grid.getDiscretizationAtPosition(1, 3) << std::endl;
    std::cout << grid.getDiscretizationAtPosition(0, 3) << std::endl;
    std::cout << grid.getDiscretizationAtPosition(0, 5) << std::endl;
}

void Scene1::onDraw(Renderer &renderer) {
    grid.drawGrid(renderer);
}
