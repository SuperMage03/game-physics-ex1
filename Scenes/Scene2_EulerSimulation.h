// File: Scene2_EulerSimulation.h
#pragma once
#include "Scene.h"
#include "glm/glm.hpp"
#include <vector>
#include <imgui.h>
#include "Renderer.h"

class Scene2_EulerSimulation : public Scene {
public:
    glm::vec3 x0, v0, x1, v1;
    float m, k, dt;

    void init() override {
        x0 = glm::vec3(0.0f, 0.0f, 0.0f);
        v0 = glm::vec3(-1.0f, 0.0f, 0.0f);
        x1 = glm::vec3(2.0f, 0.0f, 0.0f);
        v1 = glm::vec3(1.0f, 0.0f, 0.0f);
        m = 10.0f;
        k = 40.0f;
        dt = 0.005f;
    }

    void update(float deltaTime) override {
        glm::vec3 force = -k * (x1 - x0);

        v0 += (force / m) * dt;
        x0 += v0 * dt;

        v1 -= (force / m) * dt;
        x1 += v1 * dt;
    }
    void onDraw(Renderer &renderer) override {
        glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f);

        // Draw the points using small spheres
        renderer.drawSphere(x0, 0.05f, glm::vec4(color, 1.0f));
        renderer.drawSphere(x1, 0.05f, glm::vec4(color, 1.0f));

        // Draw the line connecting the two points
        renderer.drawLine(x0, x1, color);
    }

};
