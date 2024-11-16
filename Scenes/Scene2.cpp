#include "Scene2.h"
#include <imgui.h>

void Scene2::init() {
    point_registry_ = PointRegistry::getInstance();
    point_registry_->clear();
    force_registry_ = ForceRegistry::getInstance();
    force_registry_->clear();
    
    // Register Points and Force Generators
    point_registry_->add(p1);
    point_registry_->add(p2);
    force_registry_->add(p2, sfg_p1);
    force_registry_->add(p1, sfg_p2);
    // Sets integration mode to be Euler Step
    point_registry_->setIntegrationMode(PointRegistry::IntegrationMode::EULER);
}

void Scene2::simulateStep() {
    point_registry_->simulateStep(step);
}

void Scene2::onDraw(Renderer& renderer) {
    renderer.drawSphere(p1.getPosition(), 0.1f, {0.0f, 0.0f, 1.0f, 1.0f});
    renderer.drawSphere(p2.getPosition(), 0.1f, {0.0f, 0.0f, 1.0f, 1.0f});
    renderer.drawLine(p1.getPosition(), p2.getPosition(), glm::vec4(1, 0, 0, 1));
}

void Scene2::onGUI() {
    ImGui::SliderFloat("Time Step", &step, 0.001f, 0.1f);
}
