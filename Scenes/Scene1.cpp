#include "Scene1.h"
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>
void Scene1::onDraw(Renderer& renderer){
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));

    renderer.drawCube(  glm::vec3(0,0,0),
                        glm::quat(glm::vec3(pitch, roll, yaw)), // rotation now given via Euler angles
                        glm::vec3(0.5,0.5,0.5),
                        glm::vec4(1,0,0,1));
    glm::mat4 rotation = glm::toMat4(glm::quat(glm::vec3(pitch, roll, yaw)));
    glm::vec3 forward = glm::vec3(rotation * glm::vec4(0, 0, 1, 0));
    glm::vec3 right = glm::vec3(rotation * glm::vec4(1, 0, 0, 0));
    glm::vec3 up = glm::vec3(rotation * glm::vec4(0, 1, 0, 0));

    renderer.drawLine(glm::vec3(0), forward, glm::vec4(1, 0, 0, 1));
    renderer.drawLine(glm::vec3(0), right, glm::vec4(0, 1, 0, 1));
    renderer.drawLine(glm::vec3(0), up, glm::vec4(0, 0, 1, 1));
    for (auto& particle : particles){
        renderer.drawSphere(particle.position, 0.1f, particle.color);
    }
}


void Scene1::simulateStep(){
    //pitch += 0.001f;
    //roll += 0.002f;
    //yaw += 0.003f;
    //pitch += pitch_increment;
    //roll += roll_increment;
    //yaw += yaw_increment;
    glm::vec3 gravityAccel = glm::vec3(0, -9.81f, 0);
    for (auto& particle : particles){
        particle.position += 0.01f * particle.velocity;
        particle.lifetime += 0.01f;
        particle.velocity += gravityAccel * 0.01f;
    }
    particles.erase(std::remove_if(particles.begin(), particles.end(), [](const Particle& particle){
        return particle.lifetime > 1.f;
    }), particles.end());
    if(ImGui::IsKeyDown(ImGuiKey_Space))
        launchSphere();
    if(ImGui::IsKeyDown(ImGuiKey_W))
        pitch += pitch_increment;
    if(ImGui::IsKeyDown(ImGuiKey_S))
        pitch -= pitch_increment;
    if(ImGui::IsKeyDown(ImGuiKey_A))
        roll += roll_increment;
    if(ImGui::IsKeyDown(ImGuiKey_D))
        roll -= roll_increment;
    if(ImGui::IsKeyDown(ImGuiKey_Q))
        yaw += yaw_increment;
    if(ImGui::IsKeyDown(ImGuiKey_E))
        yaw -= yaw_increment;
    lastLaunch++;
}
void Scene1::launchSphere(){
    if( lastLaunch < launch_delay)
        return;
    lastLaunch = 0;
    glm::mat4 rotation = glm::toMat4(glm::quat(glm::vec3(pitch, roll, yaw)));
    glm::vec3 forward = glm::vec3(rotation * glm::vec4(0, 0, 1, 0));
    glm::vec3 right = glm::vec3(rotation * glm::vec4(1, 0, 0, 0));
    glm::vec3 up = glm::vec3(rotation * glm::vec4(0, 1, 0, 0));

    glm::vec4 color = glm::vec4(dis(gen), dis(gen), dis(gen), 1);
    float velocityMagnitude = 4.5f + dis(gen);
    glm::vec3 velocity = forward * velocityMagnitude;

    velocity += right * (dis(gen) - 0.5f) * 2.f;
    velocity += up * (dis(gen) - 0.5f) * 2.f;

    particles.push_back(Particle{glm::vec3(0), velocity, color, .0});
}

void Scene1::onGUI(){
    ImGui::SliderFloat("Pitch Increment", &pitch_increment, -0.01f, 0.01f);
    ImGui::SliderFloat("Roll Increment", &roll_increment, -0.01f, 0.01f);
    ImGui::SliderFloat("Yaw Increment", &yaw_increment, -0.01f, 0.01f);
    auto launch = ImGui::Button("Launch");

    if(launch){
        glm::mat4 rotation = glm::toMat4(glm::quat(glm::vec3(pitch, roll, yaw)));
        glm::vec3 forward = glm::vec3(rotation * glm::vec4(0, 0, 1, 0));

        particles.push_back(Particle{
            glm::vec3(0), // Initial Position
            forward * 5.f, // Initial Velocity
            glm::vec4(dis(gen), dis(gen), dis(gen), 1), // Color
            .0 // Particles are created with their own time counter set to 0
        });
    }
    ImGui::SliderInt("Launch Delay", &launch_delay, 0, 100);
}
