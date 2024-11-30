#include "Scene.h"
#include <map>

#include "SceneSingleStep.h"
#include "SceneSimulation.h"
#include "SceneCollision.h"
#include "SceneComplexSimulation.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<SceneSingleStep>()},
    {"Simulation Scene", creator<SceneSimulation>()},
    {"Collision", creator<SceneCollision>()},
    {"Complex Simulation", creator<SceneComplexSimulation>()},
    // add more Scene types here
};
