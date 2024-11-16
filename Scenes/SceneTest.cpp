#include "SceneTest.h"

void SceneTest::onDraw(Renderer &renderer) {
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
    cube1.onDraw(renderer);
    for (std::vector<Particle>::iterator iter = particles.begin(); iter != particles.end(); ++iter) {
         iter->onDraw(renderer);
    }
}

void SceneTest::simulateStep() {
    float delta = 1.f / 144.f;
    cube1.simulateStep(delta);
    for (auto& particle : particles) {
        particle.simulateStep(delta);
    }
    particleLaunchTimer.simulateStep(delta);

    launch = launch || ImGui::IsKeyDown(ImGuiKey_L);

    if (launch) {
        if (particleLaunchTimer.isAlarm()) {
            launchParticle();
            particleLaunchTimer.start();
        }
    }
    
    float particleLifetime = 2.f;
    particles.erase(
        std::remove_if(
            particles.begin(), 
            particles.end(),
            [particleLifetime](const Particle& p) {
                return p.timer >= particleLifetime;
            }
        ),
        particles.end()
    );
}

void SceneTest::onGUI() {
    ImGui::SliderFloat("Pitch rotation speed", &cube1.pitchRotationSpeed, -glm::pi<float>() / 2.f, glm::pi<float>() / 2.f);
    ImGui::SliderFloat("Roll rotation speed", &cube1.rollRotationSpeed, -glm::pi<float>() / 2.f, glm::pi<float>() / 2.f);
    ImGui::SliderFloat("Yaw rotation speed", &cube1.yawRotationSpeed, -glm::pi<float>() / 2.f, glm::pi<float>() / 2.f);

    launch = ImGui::Button("Launch");
}

void SceneTest::launchParticle() {
    Particle particle;
    particle.position = cube1.position;
    particle.velocity = random.randFRange(2.f, 6.f) * cube1.get_rotation_matrix() * glm::vec4(1.f, random.randFRange(-0.1f, 0.1f), random.randFRange(-0.1f, 0.1f), 0.f);
    particle.size = random.randFRange(0.05f, 0.15f);
    particle.color = glm::vec4(random.randFRange(0.f, 1.f), random.randFRange(0.f, 1.f), random.randFRange(0.f, 1.f), 1.f);
    particles.push_back(particle);
}
