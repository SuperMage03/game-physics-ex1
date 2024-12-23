#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SceneSingleStep.h"
#include "SceneExplicitSimulation.h"
#include "SceneImplicitSimulation.h"
#include "SceneInteractiveSimulation.h"
#include "SceneFullDemo.h"
#include "Scene3D.h"
#include "Scene3DBC.h"
#include "InteractiveSimulation2.h"

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
    {"Full Additional Features Demo", creator<SceneFullDemo>()},
    {"3D Simulation", creator<Scene3D>()},
    {"3D Simulation with BC", creator<Scene3DBC>()},
    {"Alternative Rendering Interactive Simulation", creator<InteractiveSimulation2>()}
    // add more Scene types here
};
