#include "SceneSingleStep.h"

void SceneSingleStep::init() {
    MSS = MSSBuilder::createSimpleMSS();
    MSS->setGravity(glm::vec3(0.f, 0.f, 0.f));

    std::cout << "Initial MSS state: " << std::endl
    << MSS << std::endl << std::endl
    << "Starting Euler integration..." << std::endl;

    MassSpringSystem result(*MSS);

    float delta = 1.f/2.f;

    result.eulerIntegrate(delta);

    std::cout << "MSS after Euler integration " << std::endl
    << "==========================" << std::endl
    << result << std::endl << std::endl
    << "==========================" << std::endl
    << "Starting Midpoint integration..." << std::endl;

    result = *MSS;

    result.midpointIntegrate(delta);

    std::cout << "MSS after Midpoint integration " << std::endl
    << "==========================" << std::endl
    << result << std::endl << std::endl
    << "==========================" << std::endl;
}

void SceneSingleStep::onDraw(Renderer &renderer) {
    renderer.drawWireCube(glm::vec3(0), glm::vec3(5), glm::vec3(1));
}