// File: Scene1_SingleStep.h
#include "Scene.h"

class Scene1_SingleStep : public Scene {
public:
    void init() override {
        // Initial conditions
        glm::vec3 x0(0.0f, 0.0f, 0.0f), v0(-1.0f, 0.0f, 0.0f);
        glm::vec3 x1(2.0f, 0.0f, 0.0f), v1(1.0f, 0.0f, 0.0f);
        float m = 10.0f, k = 40.0f, dt = 0.1f;

        // Euler integration
        glm::vec3 x0_euler = x0 + dt * v0;
        glm::vec3 x1_euler = x1 + dt * v1;

        // Midpoint integration
        glm::vec3 v0_mid = v0 + (dt / 2.0f) * (-k * (x1 - x0) / m);
        glm::vec3 v1_mid = v1 + (dt / 2.0f) * (-k * (x0 - x1) / m);
        glm::vec3 x0_mid = x0 + dt * v0_mid;
        glm::vec3 x1_mid = x1 + dt * v1_mid;

        // Print results
        std::cout << "Euler: x0 = (" << x0_euler.x << ", " << x0_euler.y << ", " << x0_euler.z << "), "
          << "x1 = (" << x1_euler.x << ", " << x1_euler.y << ", " << x1_euler.z << ")" << std::endl;

        std::cout << "Midpoint: x0 = (" << x0_mid.x << ", " << x0_mid.y << ", " << x0_mid.z << "), "
          << "x1 = (" << x1_mid.x << ", " << x1_mid.y << ", " << x1_mid.z << ")" << std::endl;

    }
};
