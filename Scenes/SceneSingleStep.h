#pragma once
#include "Scene.h"
#include "MSSBuilder.h"
#include "MassSpringSystem.h"

class SceneSingleStep : public Scene {
public:
    MassSpringSystem* MSS;

public:
    SceneSingleStep():
    MSS(nullptr)
    {}

    virtual void init() override;
    virtual void onDraw(Renderer &renderer) override;
};