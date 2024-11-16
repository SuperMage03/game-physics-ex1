#pragma once
#include "Scene.h"
#include "MSSBuilder.h"
#include "MassSpringSystem.h"
#include <imgui.h>

class SceneComplexSimulation: public Scene {
public:
    MassSpringSystem* MSS;
    float delta;
    bool printState;
    int integrator;
    float damping;
    float stiffness;
    bool pause;
    float impulse[3];
    bool applyImpulse;
public:
    SceneComplexSimulation():
    MSS(nullptr),
    delta(0.005f),
    printState(false),
    integrator(0),
    damping(1.f),
    stiffness(1000.f),
    pause(true),
    applyImpulse(false)
    {
        impulse[0] = 0.f;
        impulse[1] = 0.f;
        impulse[2] = 0.f;
    }

    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;
};