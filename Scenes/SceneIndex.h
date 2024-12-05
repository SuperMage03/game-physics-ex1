#include "Scene.h"
#include <map>

#include "SceneSingleStep.h"
#include "SceneSimulation.h"
#include "SceneCollision.h"
#include "SceneComplexSimulation.h"
#include "SceneRacket.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<SceneSingleStep>()},
    {"Simulation Scene", creator<SceneSimulation>()},
    {"Collision", creator<SceneCollision>()},
    {"Complex Simulation", creator<SceneComplexSimulation>()},
    {"Tennis Racket Theorem Test", creator<SceneRacket>()}
    // add more Scene types here
};
