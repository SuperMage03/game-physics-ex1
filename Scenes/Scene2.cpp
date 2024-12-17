#include "Scene2.h"
#include <imgui.h>

void Scene2::init() {
    grid.randomizePlotValues();
}

void Scene2::simulateStep() {
    if (!m_isPaused) {
        heatEquation2D.simulateStep(m_step);
    }
}

void Scene2::onDraw(Renderer &renderer) {
    grid.drawGrid(renderer, glm::vec3(0.0f), 5.0f);
}

void Scene2::onGUI() {
    ImGui::Checkbox("Pause", &m_isPaused);
    ImGui::SliderFloat("Time Step", &m_step, 0.001f, 2.0f);
    
    ImGui::SliderFloat("Diffusivity", &m_diffusivity, 0.001f, 2.0f);
    heatEquation2D.setDiffusivity(m_diffusivity);
}
