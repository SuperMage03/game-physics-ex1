#include "Scene1.h"
#include <iostream>

void Scene1::init() {
    float initialState[] = { 6.0f,  5.0f,  1.0f, -1.0f, -2.0f, -1.0f,
                             4.0f,  3.0f,  0.0f, -1.0f, -3.0f, -1.0f,
                             3.0f,  2.0f, -1.0f, -2.0f, -4.0f, -2.0f};

    HeatEquation2D heatEquation2D{0.0f, 2.0f, 0.0f, 4.0f, 0.1f, 3, 6, initialState};

    for (int i = 0; i < 10; i++) {
        heatEquation2D.simulateStep(0.1f);
    }
    std::cout << heatEquation2D.getDiscretizationAtPosition(1, 3) << std::endl;
}
