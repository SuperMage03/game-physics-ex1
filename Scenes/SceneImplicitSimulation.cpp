#include "SceneImplicitSimulation.h"
#include <imgui.h>

void SceneImplicitSimulation::init() {
    grid.gaussianRandomizePlotValues();
}

void SceneImplicitSimulation::simulateStep() {
    if (!m_isPaused) {
        HeatEquation.simulateStep(m_step);
    }
}

void SceneImplicitSimulation::onDraw(Renderer &renderer) {
    grid.drawGrid(renderer, glm::vec3(0.0f), 5.0f);
}

void SceneImplicitSimulation::onGUI() {
    ImGui::Checkbox("Pause", &m_isPaused);
    ImGui::SliderFloat("Time Step", &m_step, 0.001f, 2.0f);
    
    ImGui::SliderFloat("Diffusivity", &m_diffusivity, 0.001f, 2.0f);
    HeatEquation.setDiffusivity(m_diffusivity);
}