#include "SingleStepScene.h"
#include <iostream>
#include <vector>
#include <cmath>

void SingleStepScene::init() {
    const int m = 3;
    const int n = 6;
    const double domain_x = 2.0;
    const double domain_y = 4.0;

    const double nu = 0.1;     
    const double dt = 0.1;  
    const double dx = domain_x / (m + 1);
    const double dy = domain_y / (n + 1);

    std::vector<std::vector<double>> T_prev = {
        {6, 5, 1, -1, -2, -1},
        {4, 3, 0, -1, -3, -1},
        {3, 2, -1, -2, -4, -2}
    };

    std::vector<std::vector<double>> T_next(m, std::vector<double>(n, 0.0));

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == 0 || j == 0 || i == m - 1 || j == n - 1) {
                T_next[i][j] = 0.0;
            } else {
                double d2T_dx2 = (T_prev[i + 1][j] - 2 * T_prev[i][j] + T_prev[i - 1][j]) / (dx * dx);
                double d2T_dy2 = (T_prev[i][j + 1] - 2 * T_prev[i][j] + T_prev[i][j - 1]) / (dy * dy);
                T_next[i][j] = T_prev[i][j] + dt * nu * (d2T_dx2 + d2T_dy2);
            }
        }
    }

    std::cout << "T[1][1,3]: " << T_next[1][3] << "\n";
    std::cout << "T[1][0,3]: " << T_next[0][3] << "\n";
    std::cout << "T[1][0,5]: " << T_next[0][5] << "\n";
}
