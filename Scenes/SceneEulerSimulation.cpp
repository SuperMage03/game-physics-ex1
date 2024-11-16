#include "SceneEulerSimulation.h"


void SceneEulerSimulation::init() {
    MSS = MSSBuilder::createSimpleMSS();
    MSS->setGravity(glm::vec3(0.f, 0.f, 0.f));
}

void SceneEulerSimulation::onDraw(Renderer &renderer) {
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    MSS->onDraw(renderer);
}

void SceneEulerSimulation::simulateStep() {
    this->MSS->eulerIntegrate(this->delta);
    if (printState) {
        std::cout << *MSS << std::endl << std::endl;
    }
}

void SceneEulerSimulation::onGUI() {
    ImGui::SliderFloat("Delta", &this->delta, 0.f, 0.1f);

    printState = ImGui::Button("Print State");
}