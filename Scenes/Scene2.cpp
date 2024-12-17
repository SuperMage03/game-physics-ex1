#include "Scene2.h"

void Scene2::init() {
    grid.randomizePlotValues();
}

void Scene2::simulateStep() {
    heatEquation2D.simulateStep(0.1f);
}

void Scene2::onDraw(Renderer &renderer) {
    grid.drawGrid(renderer, glm::vec3(0.0f), 5.0f);
}
