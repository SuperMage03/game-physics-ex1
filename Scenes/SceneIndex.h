#include "Scene.h"
#include <map>

#include "Scene1.h"
#include "SceneTest.h"
#include "LevelOne.h"
#include "LevelTwo.h"
#include "LevelThree.h"

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
    // add more Scene types here
    {"Level 1", creator<LevelOne>()},
    {"Level 2", creator<LevelTwo>()},
    {"Level 3", creator<LevelThree>()}
};
