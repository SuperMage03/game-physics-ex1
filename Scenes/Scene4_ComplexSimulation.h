#include "Scene.h"
#include "glm/glm.hpp"
#include <vector>

class Scene4_ComplexSimulation : public Scene {
public:
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> velocities;
    std::vector<std::pair<int, int>> springs;
    float m, k, dt;
    bool useMidpoint, useLeapFrog;
    glm::vec3 gravity;

    void init() override {
        m = 10.0f;
        k = 40.0f;
        dt = 0.005f;
        useMidpoint = false;
        useLeapFrog = false;
        gravity = glm::vec3(0.0f, -9.81f, 0.0f);

        for (int i = 0; i < 10; ++i) {
            positions.emplace_back(i * 0.5f, 5.0f, 0.0f);
            velocities.emplace_back(0.0f, 0.0f, 0.0f);
        }

        for (int i = 0; i < 9; ++i) {
            springs.emplace_back(i, i + 1);
        }
    }

    void simulateStep() override {
        if (useLeapFrog) {
            simulateLeapFrog();
        } else if (useMidpoint) {
            simulateMidpoint();
        } else {
            simulateEuler();
        }

        applyGravity();
        handleCollisions();
    }

    void simulateEuler() {
    
        for (const auto& [i, j] : springs) {
            glm::vec3 force = -k * (positions[j] - positions[i]);

            velocities[i] += (force / m) * dt;
            velocities[j] -= (force / m) * dt;

            positions[i] += velocities[i] * dt;
            positions[j] += velocities[j] * dt;
        }
    }

    void simulateMidpoint() {
        std::vector<glm::vec3> midVelocities = velocities;

        for (const auto& [i, j] : springs) {
            glm::vec3 force = -k * (positions[j] - positions[i]);

            midVelocities[i] += (force / m) * (dt / 2.0f);
            midVelocities[j] -= (force / m) * (dt / 2.0f);
        }

        for (size_t i = 0; i < positions.size(); ++i) {
            positions[i] += midVelocities[i] * dt;
        }

        for (const auto& [i, j] : springs) {
            glm::vec3 force = -k * (positions[j] - positions[i]);

            velocities[i] += (force / m) * dt;
            velocities[j] -= (force / m) * dt;
        }
    }

    void simulateLeapFrog() {
        for (size_t i = 0; i < velocities.size(); ++i) {
            velocities[i] += gravity * (dt / 2.0f);
            positions[i] += velocities[i] * dt;
            velocities[i] += gravity * (dt / 2.0f);
        }

        for (const auto& [i, j] : springs) {
            glm::vec3 force = -k * (positions[j] - positions[i]);

            velocities[i] += (force / m) * dt;
            velocities[j] -= (force / m) * dt;
        }
    }

    void applyGravity() {
        for (size_t i = 0; i < velocities.size(); ++i) {
            velocities[i] += gravity * dt;
        }
    }

    void handleCollisions() {
        for (size_t i = 0; i < positions.size(); ++i) {
            if (positions[i].y < 0.0f) {
                positions[i].y = 0.0f; // Reset position to the ground
                velocities[i].y *= -0.5f; // Simple collision response (bounce back)
            }
        }
    }

    void onDraw(Renderer &renderer) override {
        glm::vec3 pointColor = glm::vec3(0.0f, 1.0f, 0.0f); // Green color for mass points
        glm::vec3 lineColor = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow color for springs

        for (const auto& pos : positions) {
            renderer.drawSphere(pos, 0.05f, glm::vec4(pointColor, 1.0f));
        }

        for (const auto& [i, j] : springs) {
            renderer.drawLine(positions[i], positions[j], lineColor);
        }
    }

    void onGUI() override {
        ImGui::Checkbox("Use Midpoint Integration", &useMidpoint);
        ImGui::Checkbox("Use Leap-Frog Integration", &useLeapFrog);
        ImGui::SliderFloat("Spring Constant (k)", &k, 10.0f, 100.0f);
        ImGui::SliderFloat("Time Step (dt)", &dt, 0.001f, 0.01f);
        ImGui::SliderFloat3("Gravity", &gravity.x, -20.0f, 20.0f);
    }
};

