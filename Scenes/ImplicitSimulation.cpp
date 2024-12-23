#include "ImplicitSimulation.h"
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>
#include "util/pcgsolver.h"

void ImplicitSimulation::init() {
    initializeRandomNoise(glm::vec2(1.0f, 1.0f), 16, 16);
}

void ImplicitSimulation::onGUI() {
    if(ImGui::Button("Do implicit BTCS scheme time step (or press space)") || ImGui::IsKeyDown(ImGuiKey_Space)) {
        implicitBTCSSchemeStep();
    }

    ImGui::SliderFloat("Size of time step", &deltaTTimeStepSize, 0.001f, 0.02f);
    ImGui::SliderFloat("Diffusivity v", &tempGrid.vThermalDiffusivity, 0.01f, 0.2f);

}

void ImplicitSimulation::onDraw(Renderer& renderer) {
    // draw the edges (heat value = 0.0f)
    glm::vec4 edgeColor = glm::vec4(20.0f / 60.0f, 0.0f, 1.0f - (20.0f/ 60.0f), 1.0f);
    renderer.drawCube(glm::vec3(0.0f, -0.5f + 0.25f * tempGrid.deltaX, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.5f * tempGrid.deltaX, 0.0f), edgeColor);
    renderer.drawCube(glm::vec3(0.5f - 0.25f * tempGrid.deltaY, 0.0f, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.5f * tempGrid.deltaY, 1.0f, 0.0f), edgeColor);
    renderer.drawCube(glm::vec3(0.0f, 0.5f - 0.25f * tempGrid.deltaX, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.5f * tempGrid.deltaX, 0.0f), edgeColor);
    renderer.drawCube(glm::vec3(-0.5f + 0.25f * tempGrid.deltaY, 0.0f, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.5f * tempGrid.deltaY, 1.0f, 0.0f), edgeColor);

    // draw every grid point
    for(int i = 0; i < tempGrid.mDegreesOfFreedom; i++) {
        for(int j = 0; j < tempGrid.nDegreesOfFreedom; j++) {
            renderer.drawCube(glm::vec3(-0.5f + tempGrid.deltaY + j * tempGrid.deltaY, -0.5f + tempGrid.deltaX + i * tempGrid.deltaX, 0.5f * (tempGrid.grid[i][j] / 60.0f)),
                glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(tempGrid.deltaY, tempGrid.deltaX, tempGrid.grid[i][j] / 60.0f),
                glm::vec4((tempGrid.grid[i][j] + 20.0f) / 60.0f, 0.0f, 1.0f - ((tempGrid.grid[i][j] + 20.0f)/ 60.0f), 1.0f));
        }
    }
}

void ImplicitSimulation::initializeRandomNoise(glm::vec2 domainExtent, 
    int mDegreesOfFreedom, int nDegreesOfFreedom) {

    // initializing basic properties of the grid
    tempGrid.domainExtent = domainExtent;
    tempGrid.mDegreesOfFreedom = mDegreesOfFreedom;
    tempGrid.nDegreesOfFreedom = nDegreesOfFreedom;
    tempGrid.deltaX = tempGrid.domainExtent.x / (tempGrid.mDegreesOfFreedom + 1);
    tempGrid.deltaY = tempGrid.domainExtent.y / (tempGrid.nDegreesOfFreedom + 1);
    tempGrid.vThermalDiffusivity = 0.1f;

    // initializing the grid (adding rows of random noise)
    for(int i = 0; i < tempGrid.mDegreesOfFreedom; i++) {
        std::vector<float> currentRow;
        for(int j = 0; j < tempGrid.nDegreesOfFreedom; j++) {
            // random number between -20 and 40
            currentRow.push_back(dis(gen) * 60.0f - 20.0f);
        }
        tempGrid.grid.push_back(currentRow);
    }
}

void ImplicitSimulation::explicitEulerSchemeStep() {
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

void ImplicitSimulation::implicitBTCSSchemeStep() {
    std::vector<std::vector<float>> oldGrid = tempGrid.grid;

    // initialize matrix A and vector b
    // size of matrix equals n * m, 5 values expected to be non zero per row
    SparseMatrix<float> Amatrix = SparseMatrix<float>(tempGrid.mDegreesOfFreedom * tempGrid.nDegreesOfFreedom, 5);
    Amatrix.zero();
    // size of vector b equals n * m
    std::vector<float> bVector(tempGrid.mDegreesOfFreedom * tempGrid.nDegreesOfFreedom);

    // iterate over grid and fill matrix A and vector b
    for(int i = 0; i < tempGrid.mDegreesOfFreedom; i++) {
        for(int j = 0; j < tempGrid.nDegreesOfFreedom; j++) {
            // index of current element in matrix and vector
            int currentIndex = i * tempGrid.nDegreesOfFreedom + j;

            // set matrix values
            // ----------------

            // u i,j [t+1]
            float valueAtDiagonal = 1.0f + (2.0f * tempGrid.vThermalDiffusivity * deltaTTimeStepSize * (1.0f / (tempGrid.deltaX * tempGrid.deltaX))) + 
                ((2.0f * tempGrid.vThermalDiffusivity * deltaTTimeStepSize * (1.0f / (tempGrid.deltaY * tempGrid.deltaY))));
            Amatrix.set_element(currentIndex, currentIndex, valueAtDiagonal);

            // i-1,j
            int indexAbove = (i-1) * tempGrid.nDegreesOfFreedom + j;
            // i+1,j
            int indexBelow = (i+1) * tempGrid.nDegreesOfFreedom + j;
            // i,j-1
            int indexLeft = i * tempGrid.nDegreesOfFreedom + j - 1;
            // i,j+1
            int indexRight = i * tempGrid.nDegreesOfFreedom + j + 1;

            float deltaXvalue = (-1.0f) * tempGrid.vThermalDiffusivity * deltaTTimeStepSize * (1.0f / (tempGrid.deltaX * tempGrid.deltaX));
            float deltaYvalue = (-1.0f) * tempGrid.vThermalDiffusivity * deltaTTimeStepSize * (1.0f / (tempGrid.deltaY * tempGrid.deltaY));
            if(i != 0) Amatrix.set_element(currentIndex, indexAbove, deltaXvalue);
            if(i != tempGrid.mDegreesOfFreedom - 1) Amatrix.set_element(currentIndex, indexBelow, deltaXvalue);
            if(j != 0) Amatrix.set_element(currentIndex, indexLeft, deltaYvalue);
            if(j != tempGrid.nDegreesOfFreedom - 1) Amatrix.set_element(currentIndex, indexRight, deltaYvalue);

            // set vector values
            // ----------------

            bVector[currentIndex] = oldGrid[i][j];
        }
    }

    // solve equation system
    SparsePCGSolver<float> pcgSolver;
    std::vector<float> resultVector(tempGrid.mDegreesOfFreedom * tempGrid.nDegreesOfFreedom);
    float residual;
    int iterations;
    pcgSolver.solve(Amatrix, bVector, resultVector, residual, iterations);

    // iterate over temperature grid and update every value
    for(int i = 0; i < tempGrid.mDegreesOfFreedom; i++) {
        for(int j = 0; j < tempGrid.nDegreesOfFreedom; j++) {
            // index of current element in vector
            int currentIndex = i * tempGrid.nDegreesOfFreedom + j;
            tempGrid.grid[i][j] = resultVector[currentIndex];
        }
    }
}