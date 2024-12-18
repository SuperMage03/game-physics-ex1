#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "Scene2.h"
#include "Scene3.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Single Step", creator<Scene1>()},
    {"Explicit Simulation", creator<Scene2>()},
    {"Implicit Simulation", creator<Scene3>()},
    // add more Scene types here
};
