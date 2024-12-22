#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SingleStep.h"
#include "ExplicitSimulation.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    // add more Scene types here
    {"Single Step", creator<SingleStep>()},
    {"Explicit Simulation", creator<ExplicitSimulation>()},
};
