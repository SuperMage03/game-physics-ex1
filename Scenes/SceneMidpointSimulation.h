#pragma once
#include "Scene.h"
#include "MSSBuilder.h"
#include "MassSpringSystem.h"
#include <imgui.h>

class SceneMidpointSimulation: public Scene {
public:
    MassSpringSystem* MSS;
    float delta;
    bool printState;

public:
    SceneMidpointSimulation():
    MSS(nullptr),
    delta(0.005f),
    printState(false)
    {}

    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
    virtual void simulateStep() override;
    virtual void onGUI() override;
};