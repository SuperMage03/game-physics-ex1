#pragma once
#include <map>

#include "SceneTest.h"
#include "SceneSingleStep.h"
#include "SceneEulerSimulation.h"
#include "SceneMidpointSimulation.h"
#include "SceneComplexSimulation.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    // {"DEBUG", creator<Scene>()},
    {"Demo Scene", creator<SceneTest>()},
    {"Single Step", creator<SceneSingleStep>()},
    {"Euler Simulation", creator<SceneEulerSimulation>()},
    {"Midpoint Simulation", creator<SceneMidpointSimulation>()},
    {"Complex Simulation", creator<SceneComplexSimulation>()}
    // add more Scene types here
};
