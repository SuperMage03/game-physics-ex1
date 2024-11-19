#include "Scene3.h"
#include <imgui.h>

void Scene3::init() {
    point_registry_ = PointRegistry::getInstance();
    force_registry_ = ForceRegistry::getInstance();
    
    // Register Points and Force Generators
    point_registry_->add(p1);
    point_registry_->add(p2);
    force_registry_->add(p2, sfg_p1);
    force_registry_->add(p1, sfg_p2);
    // Sets integration mode to be Midpoint Step
    point_registry_->setIntegrationMode(PointRegistry::IntegrationMode::MIDPOINT);
}

void Scene3::simulateStep() {
    point_registry_->simulateStep(step);
}

void Scene3::onDraw(Renderer& renderer) {
    renderer.drawSphere(p1.getPosition(), 0.1f, glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
    renderer.drawSphere(p2.getPosition(), 0.1f, glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
    renderer.drawLine(p1.getPosition(), p2.getPosition(), glm::vec4{1.0f, 0.0f, 0.0f, 1.0f});
}

void Scene3::onGUI() {
    ImGui::SliderFloat("Time Step", &step, 0.001f, 0.1f);
}

Scene3::~Scene3() {
    force_registry_->clear();
    point_registry_->clear();
}
