#include "SceneInteractiveSimulation.h"
#include <imgui.h>

void SceneInteractiveSimulation::init() {
    grid.gaussianRandomizePlotValues();
}

void SceneInteractiveSimulation::simulateStep() {
    if (!m_isPaused) {
        HeatEquation.simulateStep(m_step);
    }
}

void SceneInteractiveSimulation::onDraw(Renderer &renderer) {
    grid.drawGrid(renderer, glm::vec3(0.0f), 5.0f);
}

void SceneInteractiveSimulation::onGUI() {
    ImGui::Checkbox("Pause", &m_isPaused);
    ImGui::SliderFloat("Time Step", &m_step, 0.001f, 2.0f);
    
    if (ImGui::SliderInt("Row Size", &m_rowSize, 0, 100)) {
        grid.resizeGrid(m_rowSize, m_colSize);
    }
    if (ImGui::SliderInt("Col Size", &m_colSize, 0, 100)) {
        grid.resizeGrid(m_rowSize, m_colSize);
    }
    if (ImGui::SliderFloat("X Min", &m_xBoundaryMin, 0.0f, 100.0f)) {
        grid.resizeDomain(m_xBoundaryMin, m_xBoundaryMax, m_yBoundaryMin, m_yBoundaryMax);
    }
    if (ImGui::SliderFloat("X Max", &m_xBoundaryMax, 0.0f, 100.0f)) {
        grid.resizeDomain(m_xBoundaryMin, m_xBoundaryMax, m_yBoundaryMin, m_yBoundaryMax);
    }
    if (ImGui::SliderFloat("Y Min", &m_yBoundaryMin, 0.0f, 100.0f)) {
        grid.resizeDomain(m_xBoundaryMin, m_xBoundaryMax, m_yBoundaryMin, m_yBoundaryMax);
    }
    if (ImGui::SliderFloat("Y Max", &m_yBoundaryMax, 0.0f, 100.0f)) {
        grid.resizeDomain(m_xBoundaryMin, m_xBoundaryMax, m_yBoundaryMin, m_yBoundaryMax);
    }
    if (ImGui::SliderFloat("Diffusivity", &m_diffusivity, 0.001f, 2.0f)) {
        HeatEquation.setDiffusivity(m_diffusivity);
    }
}