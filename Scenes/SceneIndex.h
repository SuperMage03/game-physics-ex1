#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SingleStepScene.h"
#include "SceneExplicitSimulation.h"
#include "SceneImplicitSimulation.h"
#include "SceneInteractiveSimulation.h"
#include "Scene3DSimulation.h"


using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    //{"Demo Scene", creator<Scene1>()},
    {"Single Step", creator<SingleStepScene>()},
    {"Explicit Simulation", creator<SceneExplicitSimulation>()},
    {"Implicit Simulation", creator<SceneImplicitSimulation>()},
    {"Interactive Simulation", creator<SceneInteractiveSimulation>()},
    {"3D Simulation", creator<Scene3DSimulation>()},
    // add more Scene types here
};

