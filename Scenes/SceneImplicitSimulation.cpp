#include "SceneImplicitSimulation.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "ImGui.h"
#include "util/pcgsolver.h" 

void SceneImplicitSimulation::init() {
    m = 16;
    n = 16; 
    dx = 1.0 / (m + 1);
    dy = 1.0 / (n + 1);
    nu = 0.1;   
    dt_sim = 0.01; 

    initializeGrid();
    assembleSystem();
}

void SceneImplicitSimulation::initializeGrid() {
    T_curr.resize(m, std::vector<double>(n, 0.0));
    T_next.resize(m, std::vector<double>(n, 0.0));
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            T_curr[i][j] = static_cast<double>(rand()) / RAND_MAX; 
        }
    }
}

void SceneImplicitSimulation::assembleSystem() {
    A.resize(m * n, std::vector<double>(m * n, 0.0));
    b.resize(m * n, 0.0);

    for (int i = 1; i < m - 1; ++i) {
        for (int j = 1; j < n - 1; ++j) {
            int idx = i * n + j;
            A[idx][idx] = 1 + 2 * nu * (1 / (dx * dx) + 1 / (dy * dy)) * dt_sim;
            if (i > 0) A[idx][(i - 1) * n + j] = -nu * dt_sim / (dx * dx);
            if (i < m - 1) A[idx][(i + 1) * n + j] = -nu * dt_sim / (dx * dx);
            if (j > 0) A[idx][i * n + (j - 1)] = -nu * dt_sim / (dy * dy);
            if (j < n - 1) A[idx][i * n + (j + 1)] = -nu * dt_sim / (dy * dy);
        }
    }

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == 0 || i == m - 1 || j == 0 || j == n - 1) {
                int idx = i * n + j;
                A[idx] = std::vector<double>(m * n, 0.0);
                A[idx][idx] = 1;
                b[idx] = 0;  
            }
        }
    }
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            int idx = i * n + j;
            b[idx] = T_curr[i][j];
        }
    }
}

void SceneImplicitSimulation::solveLinearSystem() {
    SparseMatrix<double> sparse_A(m * n);
    std::vector<double> b_vec(m * n, 0.0);  

    for (int i = 1; i < m - 1; ++i) {
        for (int j = 1; j < n - 1; ++j) {
            int idx = i * n + j;
            sparse_A.set_element(idx, idx, 1 + 2 * nu * (1 / (dx * dx) + 1 / (dy * dy)) * dt_sim);
            if (i > 0) sparse_A.add_to_element(idx, (i - 1) * n + j, -nu * dt_sim / (dx * dx));
            if (i < m - 1) sparse_A.add_to_element(idx, (i + 1) * n + j, -nu * dt_sim / (dx * dx));
            if (j > 0) sparse_A.add_to_element(idx, i * n + (j - 1), -nu * dt_sim / (dy * dy));
            if (j < n - 1) sparse_A.add_to_element(idx, i * n + (j + 1), -nu * dt_sim / (dy * dy));
        }
    }

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == 0 || i == m - 1 || j == 0 || j == n - 1) {
                int idx = i * n + j;
                sparse_A.set_element(idx, idx, 1);
                b_vec[idx] = 0;  
            } else {
                b_vec[i * n + j] = T_curr[i][j];  
            }
        }
    }

    SparsePCGSolver<double> pcg_solver;
    std::vector<double> x(m * n, 0.0);  
    double relative_residual;
    int iterations;
    bool success = pcg_solver.solve(sparse_A, b_vec, x, relative_residual, iterations);

    if (success) {
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                T_next[i][j] = x[i * n + j];
            }
        }
    } else {
        std::cerr << "PCG solver failed to converge." << std::endl;
    }
}

void SceneImplicitSimulation::simulateStep() {
    solveLinearSystem();
}

void SceneImplicitSimulation::onDraw(Renderer &renderer) {
    std::vector<float> image(m * n, 0.0f);
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            image[i * n + j] = static_cast<float>(T_next[i][j]);
        }
    }

    renderer.drawImage(image, m, n, Colormap("hot")); 
}

void SceneImplicitSimulation::onGUI() {
    float nu_float = static_cast<float>(nu);
    float dt_sim_float = static_cast<float>(dt_sim);
    if (ImGui::SliderFloat("Thermal Diffusivity (nu)", &nu_float, 0.01f, 1.0f)) {
         nu = static_cast<double>(nu_float);
    }
    if (ImGui::SliderFloat("Time Step Size (dt)", &dt_sim_float, 0.001f, 0.1f)) {
        dt_sim = static_cast<double>(dt_sim_float);
    }
}
