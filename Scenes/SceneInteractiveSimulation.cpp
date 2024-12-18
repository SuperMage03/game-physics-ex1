#include "SceneInteractiveSimulation.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "ImGui.h"
#include "util/pcgsolver.h"  

void SceneInteractiveSimulation::init() {
    m = 16;  
    n = 16;  
    dx = 1.0 / (m + 1);
    dy = 1.0 / (n + 1);
    nu = 0.1;  
    dt_sim = 0.01;
    isExplicitMethod = true; 

    initializeGrid();
}

void SceneInteractiveSimulation::initializeGrid() {
    T_curr.resize(m, std::vector<double>(n, 0.0));
    T_next.resize(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            T_curr[i][j] = static_cast<double>(rand()) / RAND_MAX;
        }
    }
}

void SceneInteractiveSimulation::updateTemperature() {
    if (isExplicitMethod) {
        for (int i = 1; i < m - 1; ++i) {
            for (int j = 1; j < n - 1; ++j) {
                T_next[i][j] = T_curr[i][j] + nu * dt_sim * (
                    (T_curr[i + 1][j] - 2 * T_curr[i][j] + T_curr[i - 1][j]) / (dx * dx) +
                    (T_curr[i][j + 1] - 2 * T_curr[i][j] + T_curr[i][j - 1]) / (dy * dy)
                );
            }
        }
    } else {
        // Use implicit (Crank-Nicolson or BTCS) method
        // Assemble the system and solve using the PCG solver as shown in the previous scenes
        // For simplicity, you can use Crank-Nicolson method as explained in Scene 3
        // (Refer to `solveLinearSystem` function and adapt it here)
    }
    std::swap(T_curr, T_next);
}

void SceneInteractiveSimulation::simulateStep() {
    updateTemperature();
}

void SceneInteractiveSimulation::onDraw(Renderer &renderer) {
    std::vector<float> image(m * n, 0.0f);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            image[i * n + j] = static_cast<float>(T_curr[i][j]);
        }
    }
    renderer.drawImage(image, m, n, Colormap("hot"));
}

void SceneInteractiveSimulation::onGUI() {
    if (ImGui::SliderInt("Grid Rows (m)", &m, 8, 64)) {
        n = m; 
        initializeGrid();
    }

    if (ImGui::SliderInt("Grid Columns (n)", &n, 8, 64)) {
        initializeGrid();
    }

    float nu_float = static_cast<float>(nu);
    float dt_sim_float = static_cast<float>(dt_sim);
    if (ImGui::SliderFloat("Thermal Diffusivity (nu)", &nu_float, 0.01f, 1.0f)) {
         nu = static_cast<double>(nu_float);
    }
    if (ImGui::SliderFloat("Time Step Size (dt)", &dt_sim_float, 0.001f, 0.1f)) {
        dt_sim = static_cast<double>(dt_sim_float);
    }

    if (ImGui::Checkbox("Use Explicit Method", &isExplicitMethod)) {
        isExplicitMethod = !isExplicitMethod; 
    }

    ImVec2 mousePos = ImGui::GetMousePos();
    float gridWidth = ImGui::GetWindowSize().x;
    float gridHeight = ImGui::GetWindowSize().y;

    if (ImGui::IsMouseHoveringRect(ImVec2(0, 0), ImVec2(gridWidth, gridHeight), true)) {
        int i = static_cast<int>((mousePos.x / gridWidth) * m); 
        int j = static_cast<int>((mousePos.y / gridHeight) * n); 
        setTemperatureAtMouse(100.0); 
    }
}


void SceneInteractiveSimulation::setTemperatureAtMouse(double temp) {
    ImVec2 mousePos = ImGui::GetMousePos();
    int i = static_cast<int>((mousePos.x / (ImGui::GetWindowSize().x / m)));
    int j = static_cast<int>((mousePos.y / (ImGui::GetWindowSize().y / n)));

    if (i >= 0 && i < m && j >= 0 && j < n) {
        T_curr[i][j] = temp;  
    }
}
