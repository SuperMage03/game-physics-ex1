#include "Scene4.h"
#include <imgui.h>

// File scope helper function for drawing the axis along the origin
static void drawAxis(Renderer& renderer) {
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{1.0f, 0.0f, 0.0f}, glm::vec4{1.0f, 0.0f, 0.0f, 1.0f});
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{0.0f, 1.0f, 0.0f}, glm::vec4{0.0f, 1.0f, 0.0f, 1.0f});
    renderer.drawLine(glm::vec3(0.0f), glm::vec3{0.0f, 0.0f, 1.0f}, glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
}

Scene4::SpringForcePair::SpringForcePair(Point &a, Point &b, const float &stiffness): 
    a{a}, b{b},
    spring_force_from_a{a, stiffness, glm::distance(a.getPosition(), b.getPosition())}, 
    spring_force_from_b{b, stiffness, glm::distance(a.getPosition(), b.getPosition())} {}

Scene4::SpringForcePair::SpringForcePair(Point &a, Point &b, const float &stiffness, const float &initial_length): 
    a{a}, b{b},
    spring_force_from_a{a, stiffness, initial_length}, 
    spring_force_from_b{b, stiffness, initial_length} {}

void Scene4::SpringForcePair::registerSpringForcePair(ForceRegistry &force_registry) {
    force_registry.add(a, spring_force_from_b);
    force_registry.add(b, spring_force_from_a);
}

void Scene4::GenerateIcosahedron() {
    for (int i = 0; i < ICOSAHEDRON_VERTEX_COUNT; i++) {
        icosahedron_vertices_[i] = std::make_unique<Point>(ICOSAHEDRON_VERTICES[i], glm::vec3(0), glm::vec3(0), DEFAULT_POINT_MASS, 0.0f);
        // Register the created point
        point_registry_->add(*(icosahedron_vertices_[i]));
        // Adds gravity to all points
        force_registry_->add(*(icosahedron_vertices_[i]), gravity_force_generator_);
        // Adds Collision detection with bounding box to all points
        collision_registry_->add(*(icosahedron_vertices_[i]), bounding_box_collision_generator_);
    }

    for (int i = 0; i < ICOSAHEDRON_EDGE_COUNT; i++) {
        const auto& [vertex_a, vertex_b] = ICOSAHEDRON_EDGES[i];
        if (!icosahedron_vertices_[vertex_a] || !icosahedron_vertices_[vertex_b]) continue;
        icosahedron_edges_[i] = std::make_unique<SpringForcePair>(*(icosahedron_vertices_[vertex_a]), *(icosahedron_vertices_[vertex_b]), SPRING_STIFFNESS);
        // Registers the spring force pair to the force registry
        icosahedron_edges_[i]->registerSpringForcePair(*force_registry_);
    }
}

void Scene4::init() {
    point_registry_ = PointRegistry::getInstance();
    point_registry_->clear();
    force_registry_ = ForceRegistry::getInstance();
    force_registry_->clear();
    collision_registry_ = CollisionRegistry::getInstance();
    collision_registry_->clear();
    
    GenerateIcosahedron();
}

void Scene4::simulateStep() {
    point_registry_->simulateStep(step);
}

void Scene4::onDraw(Renderer& renderer) {
    for (int i = 0; i < ICOSAHEDRON_VERTEX_COUNT; i++) {
        if (!icosahedron_vertices_[i]) continue;
        renderer.drawSphere(icosahedron_vertices_[i]->getPosition(), 0.1f, glm::vec4{0.0f, 1.0f, 0.0f, 1.0f});
    }

    for (int i = 0; i < ICOSAHEDRON_EDGE_COUNT; i++) {
        const auto& [vertex_a, vertex_b] = ICOSAHEDRON_EDGES[i];
        if (!icosahedron_vertices_[vertex_a] || !icosahedron_vertices_[vertex_b]) continue;
        renderer.drawLine(icosahedron_vertices_[vertex_a]->getPosition(), icosahedron_vertices_[vertex_b]->getPosition(), glm::vec4{0.0f, 0.0f, 1.0f, 1.0f});
    }

    drawAxis(renderer);

    // Draws the bounding box
    renderer.drawWireCube(BOUNDING_BOX_POSITION, BOUNDING_BOX_SCALE, glm::vec3(1));
}

void Scene4::onGUI() {
    ImGui::SliderFloat("Time Step", &step, 0.001f, 0.1f);
}
