#include "ExplicitSimulation.h"
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>

void ExplicitSimulation::init() {
    initializeRandomNoise(glm::vec2(1.0f, 1.0f), 16, 16);
}

void ExplicitSimulation::onGUI() {
    if(ImGui::Button("Do explicit Euler scheme time step (or press space)") || ImGui::IsKeyDown(ImGuiKey_Space)) {
        explicitEulerSchemeStep();
    }

    ImGui::SliderFloat("Size of time step", &deltaTTimeStepSize, 0.001f, 0.02f);
    ImGui::SliderFloat("Diffusivity v", &tempGrid.vThermalDiffusivity, 0.01f, 0.2f);

}

void ExplicitSimulation::onDraw(Renderer& renderer) {
    // draw the edges (heat value = 0.0f)
    glm::vec4 edgeColor = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
    renderer.drawCube(glm::vec3(0.0f, -0.5f + 0.25f * tempGrid.deltaX, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.5f * tempGrid.deltaX, 0.0f), edgeColor);
    renderer.drawCube(glm::vec3(0.5f - 0.25f * tempGrid.deltaY, 0.0f, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.5f * tempGrid.deltaY, 1.0f, 0.0f), edgeColor);
    renderer.drawCube(glm::vec3(0.0f, 0.5f - 0.25f * tempGrid.deltaX, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.5f * tempGrid.deltaX, 0.0f), edgeColor);
    renderer.drawCube(glm::vec3(-0.5f + 0.25f * tempGrid.deltaY, 0.0f, 0.0f), glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.5f * tempGrid.deltaY, 1.0f, 0.0f), edgeColor);

    // draw every grid point
    for(int i = 0; i < tempGrid.mDegreesOfFreedom; i++) {
        for(int j = 0; j < tempGrid.nDegreesOfFreedom; j++) {
            renderer.drawCube(glm::vec3(-0.5f + tempGrid.deltaY + j * tempGrid.deltaY, -0.5f + tempGrid.deltaX + i * tempGrid.deltaX, 0.5f * tempGrid.grid[i][j]),
                glm::highp_quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(tempGrid.deltaY, tempGrid.deltaX, tempGrid.grid[i][j]),
                glm::vec4(tempGrid.grid[i][j], 0.0f, 1.0f - tempGrid.grid[i][j], 1.0f));
        }
    }
}

void ExplicitSimulation::initializeRandomNoise(glm::vec2 domainExtent, 
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
            // random number between 0 and 1
            currentRow.push_back(dis(gen));
        }
        tempGrid.grid.push_back(currentRow);
    }
}

void ExplicitSimulation::explicitEulerSchemeStep() {
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