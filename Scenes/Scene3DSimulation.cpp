#include "Scene3DSimulation.h"
#include "util/pcgsolver.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "ImGui.h"

void Scene3DSimulation::init() {
    m = 16; 
    n = 16; 
    p = 16;  

    dx = 1.0 / (m + 1);
    dy = 1.0 / (n + 1);
    dz = 1.0 / (p + 1);

    nu = 0.1;     
    dt_sim = 0.01; 

    isExplicitMethod = true; 

    initializeGrid();
}

void Scene3DSimulation::initializeGrid() {
    T_curr.resize(m, std::vector<std::vector<double>>(n, std::vector<double>(p, 0.0)));
    T_next.resize(m, std::vector<std::vector<double>>(n, std::vector<double>(p, 0.0)));

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < p; ++k) {
                T_curr[i][j][k] = static_cast<double>(rand()) / RAND_MAX;
            }
        }
    }
}

void Scene3DSimulation::simulateExplicitStep() {
    for (int i = 1; i < m - 1; ++i) {
        for (int j = 1; j < n - 1; ++j) {
            for (int k = 1; k < p - 1; ++k) {
                T_next[i][j][k] = T_curr[i][j][k] + nu * dt_sim * (
                    (T_curr[i + 1][j][k] - 2 * T_curr[i][j][k] + T_curr[i - 1][j][k]) / (dx * dx) +
                    (T_curr[i][j + 1][k] - 2 * T_curr[i][j][k] + T_curr[i][j - 1][k]) / (dy * dy) +
                    (T_curr[i][j][k + 1] - 2 * T_curr[i][j][k] + T_curr[i][j][k - 1]) / (dz * dz)
                );
            }
        }
    }
    std::swap(T_curr, T_next);
}

void Scene3DSimulation::simulateImplicitStep() {
    SparseMatrix<double> sparse_A(m * n * p);
    std::vector<double> b_vec(m * n * p, 0.0);
    std::vector<double> x(m * n * p, 0.0);

    double coeff = nu * dt_sim * (1 / (dx * dx) + 1 / (dy * dy) + 1 / (dz * dz));

    for (int i = 1; i < m - 1; ++i) {
        for (int j = 1; j < n - 1; ++j) {
            for (int k = 1; k < p - 1; ++k) {
                int idx = (i * n + j) * p + k;

                sparse_A.set_element(idx, idx, 1 + 2 * coeff);

                if (i > 0) sparse_A.add_to_element(idx, ((i - 1) * n + j) * p + k, -nu * dt_sim / (dx * dx));
                if (i < m - 1) sparse_A.add_to_element(idx, ((i + 1) * n + j) * p + k, -nu * dt_sim / (dx * dx));
                if (j > 0) sparse_A.add_to_element(idx, (i * n + (j - 1)) * p + k, -nu * dt_sim / (dy * dy));
                if (j < n - 1) sparse_A.add_to_element(idx, (i * n + (j + 1)) * p + k, -nu * dt_sim / (dy * dy));
                if (k > 0) sparse_A.add_to_element(idx, (i * n + j) * p + (k - 1), -nu * dt_sim / (dz * dz));
                if (k < p - 1) sparse_A.add_to_element(idx, (i * n + j) * p + (k + 1), -nu * dt_sim / (dz * dz));
            }
        }
    }
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < p; ++k) {
                int idx = (i * n + j) * p + k;
                if (i == 0 || i == m - 1 || j == 0 || j == n - 1 || k == 0 || k == p - 1) {
                    sparse_A.set_element(idx, idx, 1);
                    b_vec[idx] = 0;
                } else {
                    b_vec[idx] = T_curr[i][j][k];
                }
            }
        }
    }
    SparsePCGSolver<double> pcg_solver;
    double relative_residual;
    int iterations;

    bool success = pcg_solver.solve(sparse_A, b_vec, x, relative_residual, iterations);

    if (success) {
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                for (int k = 0; k < p; ++k) {
                    T_next[i][j][k] = x[(i * n + j) * p + k];
                }
            }
        }
        std::swap(T_curr, T_next); 
    } else {
        std::cerr << "PCG solver failed to converge." << std::endl;
    }
}

void Scene3DSimulation::simulateStep() {
    if (isExplicitMethod) {
        simulateExplicitStep();
    } else {
        simulateImplicitStep();
    }
}

void Scene3DSimulation::onDraw(Renderer &renderer) {
    std::vector<float> slice(m * n, 0.0f);
    int z_slice = p / 2; 

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            slice[i * n + j] = static_cast<float>(T_curr[i][j][z_slice]);
        }
    }

    renderer.drawImage(slice, m, n, Colormap("hot"));
}

void Scene3DSimulation::onGUI() {
    float nu_float = static_cast<float>(nu);
    float dt_sim_float = static_cast<float>(dt_sim);
    if (ImGui::SliderInt("Grid Size (m)", &m, 8, 64)) initializeGrid();
    if (ImGui::SliderInt("Grid Size (n)", &n, 8, 64)) initializeGrid();
    if (ImGui::SliderInt("Grid Size (p)", &p, 8, 64)) initializeGrid();
    if (ImGui::SliderFloat("Thermal Diffusivity (nu)", &nu_float, 0.01f, 1.0f)) {}
    if (ImGui::SliderFloat("Time Step Size (dt)", &dt_sim_float, 0.001f, 0.1f)) {}
    if (ImGui::Checkbox("Use Explicit Method", &isExplicitMethod)) {}
}
