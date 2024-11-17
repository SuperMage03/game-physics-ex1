#include "Scene.h"
#include <map>
//#include "SceneRegistry.h"
#include "Scene1_SingleStep.h"
#include "Scene2_EulerSimulation.h"
#include "Scene3_MidpointSimulation.h"
#include "Scene4_ComplexSimulation.h"
//REGISTER_SCENE(Scene1_SingleStep, "Single Step");

#include "Scene1.h"

using SceneCreator = std::function<std::unique_ptr<Scene>()>;

template <typename T>
SceneCreator creator()
{
    return []()
    { return std::make_unique<T>(); };
}

std::map<std::string, SceneCreator> scenesCreators = {
    {"Demo Scene", creator<Scene1>()},
    {"Demo Scene 2", creator<Scene1_SingleStep>()},
    {"Demo Scene 3", creator<Scene2_EulerSimulation>()},
    {"Demo Scene 4", creator<Scene3_MidpointSimulation>()},
    {"Demo Scene 5", creator<Scene4_ComplexSimulation>()}
    // add more Scene types here
};
