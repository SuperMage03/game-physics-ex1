#include "SceneExplicitSimulation.h"
#include <cmath>
#include <cstdlib>
#include "ImGui.h"

void SceneExplicitSimulation::init() {
    m = 16;
    n = 16;
    dx = 1.0 / (m + 1);
    dy = 1.0 / (n + 1);
    nu = 0.1;
    dt_sim = 0.01; 

    initializeGrid();
}

void SceneExplicitSimulation::initializeGrid() {
    T_curr.resize(m, std::vector<double>(n, 0.0));
    T_next.resize(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            T_curr[i][j] = static_cast<double>(rand()) / RAND_MAX; 
        }
    }
}

void SceneExplicitSimulation::updateTemperature() {
    for (int i = 1; i < m - 1; ++i) {
        for (int j = 1; j < n - 1; ++j) {
            double d2T_dx2 = (T_curr[i + 1][j] - 2 * T_curr[i][j] + T_curr[i - 1][j]) / (dx * dx);
            double d2T_dy2 = (T_curr[i][j + 1] - 2 * T_curr[i][j] + T_curr[i][j - 1]) / (dy * dy);
            T_next[i][j] = T_curr[i][j] + dt_sim * nu * (d2T_dx2 + d2T_dy2);
        }
    }
    std::swap(T_curr, T_next);
}

void SceneExplicitSimulation::simulateStep() {
    updateTemperature();
}

void SceneExplicitSimulation::onDraw(Renderer &renderer) {
    std::vector<float> image(m * n, 0.0f);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            image[i * n + j] = static_cast<float>(T_curr[i][j]); 
        }
    }

    renderer.drawImage(image, m, n, Colormap("hot")); 
}

void SceneExplicitSimulation::onGUI() {
    float nu_float = static_cast<float>(nu);
    float dt_sim_float = static_cast<float>(dt_sim);

    if (ImGui::SliderFloat("Thermal Diffusivity (nu)", &nu_float, 0.01f, 1.0f)) {
        nu = static_cast<double>(nu_float); 
    }
    if (ImGui::SliderFloat("Time Step Size (dt)", &dt_sim_float, 0.001f, 0.1f)) {
        dt_sim = static_cast<double>(dt_sim_float); 
    }
    if (ImGui::SliderInt("Grid Rows (m)", &m, 8, 64) || ImGui::SliderInt("Grid Columns (n)", &n, 8, 64)) {
        initializeGrid(); 
    }
}




