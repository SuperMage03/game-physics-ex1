#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SceneSingleStep.h"
#include "SceneExplicitSimulation.h"
#include "SceneImplicitSimulation.h"
#include "SceneInteractiveSimulation.h"
#include "SceneFullDemo.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    // {"Demo Scene", creator<Scene1>()},
    {"Single Step", creator<SceneSingleStep>()},
    {"Explicit Simulation", creator<SceneExplicitSimulation>()},
    {"Implicit Simulation", creator<SceneImplicitSimulation>()},
    {"Interactive Simulation", creator<SceneInteractiveSimulation>()},
    {"Full Additional Features Demo", creator<SceneFullDemo>()}
    // add more Scene types here
};
