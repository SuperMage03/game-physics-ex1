
#include "Scene.h"
#include "glm/glm.hpp"
class Scene3_MidpointSimulation : public Scene {
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

        glm::vec3 v0_mid = v0 + (dt / 2.0f) * (force / m);
        glm::vec3 v1_mid = v1 - (dt / 2.0f) * (force / m);

        x0 += v0_mid * dt;
        x1 += v1_mid * dt;

        v0 += (force / m) * dt;
        v1 -= (force / m) * dt;
    }

    void onDraw(Renderer &renderer) override {
        glm::vec3 color = glm::vec3(0.0f, 0.0f, 1.0f);

        renderer.drawSphere(x0, 0.05f, glm::vec4(color, 1.0f));
        renderer.drawSphere(x1, 0.05f, glm::vec4(color, 1.0f));

        renderer.drawLine(x0, x1, color);
    }
};
