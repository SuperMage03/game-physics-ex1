#include "Scene3.h"
#include <imgui.h>

void Scene3::init() {
    grid.randomizePlotValues();
}

void Scene3::simulateStep() {
    if (!m_isPaused) {
        heatEquation2D.simulateStep(m_step);
    }
}

void Scene3::onDraw(Renderer &renderer) {
    grid.drawGrid(renderer, glm::vec3(0.0f), 5.0f);
}

void Scene3::onGUI() {
    ImGui::Checkbox("Pause", &m_isPaused);
    ImGui::SliderFloat("Time Step", &m_step, 0.001f, 2.0f);
    
    ImGui::SliderFloat("Diffusivity", &m_diffusivity, 0.001f, 2.0f);
    heatEquation2D.setDiffusivity(m_diffusivity);
}
