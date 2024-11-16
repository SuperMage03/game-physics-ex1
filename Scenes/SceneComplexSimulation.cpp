#include "SceneComplexSimulation.h"

void SceneComplexSimulation::init() {
    this->MSS = MSSBuilder::createComplexMSS();
    this->MSS->setGravity(glm::vec3(0.f, 0.f, -9.81f));
}

void SceneComplexSimulation::onDraw(Renderer &renderer) {
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    this->MSS->onDraw(renderer);
}

void SceneComplexSimulation::simulateStep() {
    if (this->applyImpulse) {
        glm::vec3 impulse = glm::vec3(this->impulse[0], this->impulse[1], this->impulse[2]);
        this->MSS->applyImpulseToAllMP(impulse);
    }
    if (!pause) {
        switch (integrator)
        {
        case 0:
            this->MSS->eulerIntegrate(this->delta);
            break;
        case 1:
            this->MSS->midpointIntegrate(this->delta);
            break;
        case 2:
            this->MSS->leapfrogIntegrate(this->delta);
        default:
            this->MSS->eulerIntegrate(this->delta);
            break;
        }
    }
    if (this->printState) {
        std::cout << *MSS << std::endl << std::endl;
    }
}

void SceneComplexSimulation::onGUI() {
    ImGui::SliderFloat("Delta", &this->delta, 0.001f, 0.1f);
    const char* integratorNames[] = {"Euler", "Midpoint", "Leapfrog"};
    ImGui::Combo("Integrator", &this->integrator, integratorNames, 3);
    this->printState = ImGui::Button("Print State");
    ImGui::InputFloat("Damping", &this->damping, 0.01);
    if (ImGui::IsItemEdited()) {
        this->MSS->setDampingToAllMP(this->damping);
    }
    ImGui::InputFloat("Stiffness", &this->stiffness, 25);
    if (ImGui::IsItemEdited()) {
        this->MSS->setStiffnessToAllS(this->stiffness);
    }
    ImGui::Separator();
    ImGui::DragFloat3("Impulse", this->impulse, 1.0, -25.f, 25.f);
    this->applyImpulse = ImGui::Button("Apply impulse to body");
    ImGui::Separator();
    ImGui::Checkbox("Pause", &this->pause);
}