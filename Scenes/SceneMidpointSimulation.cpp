#include "SceneMidpointSimulation.h"


void SceneMidpointSimulation::init() {
    MSS = MSSBuilder::createSimpleMSS();
    MSS->setGravity(glm::vec3(0.f, 0.f, 0.f));
}

void SceneMidpointSimulation::onDraw(Renderer &renderer) {
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    MSS->onDraw(renderer);
}

void SceneMidpointSimulation::simulateStep() {
    this->MSS->midpointIntegrate(this->delta);
    if (printState) {
        std::cout << *MSS << std::endl << std::endl;
    }
}

void SceneMidpointSimulation::onGUI() {
    ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1f);

    printState = ImGui::Button("Print State");
}