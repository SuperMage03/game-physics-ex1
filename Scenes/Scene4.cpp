#include "Scene4.h"
#include <imgui.h>

void Scene4::init() {
    point_registry_ = PointRegistry::getInstance();
    point_registry_->clear();
    force_registry_ = ForceRegistry::getInstance();
    force_registry_->clear();
}

void Scene4::simulateStep() {
    point_registry_->simulateStep(step);
}

void Scene4::onDraw(Renderer& renderer) {
}

void Scene4::onGUI() {
    ImGui::SliderFloat("Time Step", &step, 0.001f, 0.1f);
}
