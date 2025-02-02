#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SceneTest.h"
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
    {"Scene Test", creator<SceneTest>()},
    {"Scene Full Demo", creator<SceneFullDemo>()},
    // add more Scene types here
};
