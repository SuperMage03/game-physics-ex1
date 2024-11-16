#pragma once
#include "Scene.h"
#include <imgui.h>
#include <random>
#include <iterator>

static const glm::vec3 GRAVITY = glm::vec3(0.f, 0.f, -9.81f);

class Random {
public:
    std::random_device rd;
    std::mt19937 gen;
    
    Random() : gen(rd())
    {}

    float randFRange(float from, float to) {
        std::uniform_real_distribution<float> dis(from, to);
        return dis(gen);
    }
};

class Cube {
public:
    // position
    glm::vec3 position;
    // pitch, roll and yaw angles
    float pitch;
    float roll;
    float yaw;

    // pith, roll and yaw angle rotation speeds
    float pitchRotationSpeed;
    float rollRotationSpeed;
    float yawRotationSpeed;

    //
    bool dbgFlag;

    Cube() {
        position = glm::vec3(0.f);
        pitch = 0.f;
        roll = 0.f;
        yaw = 0.f;
        pitchRotationSpeed = 0.f;
        rollRotationSpeed = 0.f;
        yawRotationSpeed = 0.f;
        dbgFlag = false;
    }

    void normalizeAngles() {
        //imagine a method here
    }

    void simulateStep(float delta) {
        pitch += delta * pitchRotationSpeed;
        roll += delta * rollRotationSpeed;
        yaw += delta * yawRotationSpeed;
        normalizeAngles();
    }

    void onDraw(Renderer &renderer) {
        renderer.drawCube(
        position,
        glm::quat(glm::vec3(pitch, roll, yaw)),
        glm::vec3(0.5f, 0.5f, 0.5f),
        glm::vec4(1.f, 0.5f, 0.f, 1.f)
        );
        if (dbgFlag) {
            glm::mat4 rotation = get_rotation_matrix();
            glm::vec3 forward = glm::vec3(rotation * glm::vec4(0.f, 0.f, 1.f, 0.f));
            glm::vec3 right = glm::vec3(rotation * glm::vec4(1.f, 0.f, 0.f, 0.f));
            glm::vec3 up = glm::vec3(rotation * glm::vec4(0.f, 1.f, 0.f, 0.f));

            renderer.drawLine(position, forward, glm::vec4(1.f, 0.f, 0.f, 1.f));
            renderer.drawLine(position, right, glm::vec4(0.f, 1.f, 0.f, 1.f));
            renderer.drawLine(position, up, glm::vec4(0.f, 0.f, 1.f, 1.f));
        }
    }

    glm::mat4 get_rotation_matrix() {
        return glm::mat4_cast(glm::quat(glm::vec3(pitch, roll, yaw)));
    }

};

class Particle {
public:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec4 color;
    float size;
    float timer;

    Particle() {
        position = glm::vec3(0.f);
        velocity = glm::vec3(0.f);
        color = glm::vec4(1.f);
        size = 0.1f;
        timer = 0.f;
    }

    void simulateStep(float delta) {
        velocity += delta * GRAVITY;
        position += delta * velocity;
        timer += delta;
    }

    void onDraw(Renderer &renderer) {
        renderer.drawSphere(position, size, color);
    }
};

class Timer {
private:
    float t;
    float waitTime;
    bool oneShot;
    bool stopped;

    bool alarm;

public:
    Timer() {
        t = 0.f;
        waitTime = 1.f;
        oneShot = true;
        stopped = true;
        alarm = false;
    }

    const bool isAlarm() {return alarm;}

    const bool isStopped() {return stopped;}

    const bool isOneshot() {return oneShot;}

    const float getWaitTime() {return waitTime;}

    const float getT() {return t;}

    void setWaitTime(float wt) {
        stop();
        waitTime = wt;
    }

    void setOneShot(bool os) {
        stop();
        oneShot = os;
    }

    Timer (float wt, bool os = true) : Timer() {
        waitTime = wt;
        oneShot = os;
    }

    void start() {
        stopped = false;
        alarm = false;
        t = 0.f;
    }
    
    void stop() {
        stopped = true;
        alarm = false;
    }

    void resume() {
        stopped = false;
    }

    void simulateStep(float delta) {
        if (stopped) return;
        t += delta;
        if (oneShot) {
            if (t >= waitTime) {
                alarm = true;
                stopped = true;
            }
        }
        else {
            if (t >= waitTime) {
                alarm = true;
                while (t >= waitTime) {
                    t -= waitTime;
                }
            }
            else {
                alarm = false;
            }
        }
    }
};

class SceneTest : public Scene {
public:
    Cube cube1;
    std::vector<Particle> particles;
    Timer particleLaunchTimer;
    Random random;
    bool launch;
    

    SceneTest() :
    particleLaunchTimer(0.3f)
    {
        cube1.dbgFlag = true;
        launch = false;
        particleLaunchTimer.start();
    }

    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;
    void launchParticle();
};



// QUESTIONS
// Why ambient light does not take body color into account?
// How to get frame generation timer value (_delta)