#include "SingleStep.h"
#include <imgui.h>

void SingleStep::init() {
    initializeExampleGrid();
    implicitEulerSchemeStep();

    std::cout << "T1,3 [1] = " << tempGrid.grid[1][3] << std::endl;
    std::cout << "T0,3 [1] = " << tempGrid.grid[0][3] << std::endl;
    std::cout << "T0,5 [1] = " << tempGrid.grid[0][5] << std::endl;
}

void SingleStep::initializeExampleGrid() {
    // initializing basic properties of the grid
    tempGrid.domainExtent = glm::vec2(2.0f, 4.0f);
    tempGrid.mDegreesOfFreedom = 3;
    tempGrid.nDegreesOfFreedom = 6;
    tempGrid.deltaX = tempGrid.domainExtent.x / (tempGrid.mDegreesOfFreedom + 1);
    tempGrid.deltaY = tempGrid.domainExtent.y / (tempGrid.nDegreesOfFreedom + 1);
    tempGrid.vThermalDiffusivity = 0.1f;

    // initializing the grid (adding rows)
    tempGrid.grid.push_back(std::vector<float> {
        6.0f, 5.0f, 1.0f, -1.0f, -2.0f, -1.0f
    });
    tempGrid.grid.push_back(std::vector<float> {
        4.0f, 3.0f, 0.0f, -1.0f, -3.0f, -1.0f
    });
    tempGrid.grid.push_back(std::vector<float> {
        3.0f, 2.0f, -1.0f, -2.0f, -4.0f, -2.0f
    });
}

void SingleStep::implicitEulerSchemeStep() {
    std::vector<std::vector<float>> oldGrid = tempGrid.grid;
    // iterate over temperature grid and update every value
    for(int i = 0; i < tempGrid.mDegreesOfFreedom; i++) {
        for(int j = 0; j < tempGrid.nDegreesOfFreedom; j++) {
            // [i][j]
            float valueAtIJ = oldGrid[i][j];
            // [i+1][j]
            float valueBelow = (i+1) < tempGrid.mDegreesOfFreedom ? oldGrid[i+1][j] : 0.0f;
            // [i-1][j]
            float valueAbove = (i-1) >= 0 ? oldGrid[i-1][j] : 0.0f;
            // [i][j+1]
            float valueRight = (j+1) < tempGrid.nDegreesOfFreedom ? oldGrid[i][j+1] : 0.0f;
            // [i][j-1]
            float valueLeft = (j-1) >= 0 ? oldGrid[i][j-1] : 0.0f;

            tempGrid.grid[i][j] = deltaTTimeStepSize * tempGrid.vThermalDiffusivity * 
                (((valueBelow - (2 * valueAtIJ) + valueAbove)/(tempGrid.deltaX * tempGrid.deltaX)) +
                ((valueRight - (2 * valueAtIJ) + valueLeft)/(tempGrid.deltaY * tempGrid.deltaY))) +
                valueAtIJ;
        }
    }
}