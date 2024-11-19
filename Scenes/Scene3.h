#include "Scene.h"
#include "Point.h"
#include "ForceGenerator.h"
#include "AssignmentSpecificValues.h"

class Scene3 : public Scene {
private:
    float step = 0.005f;

    // Creating Points
    Point p1{INITIAL_P1_POSITION, INITIAL_P1_VELOCITY, glm::vec3(0.0f), DEFAULT_POINT_MASS, 0.0f};
    Point p2{INITIAL_P2_POSITION, INITIAL_P2_VELOCITY, glm::vec3(0.0f), DEFAULT_POINT_MASS, 0.0f};
    // Creating Force Generators
    SpringForceGenerator sfg_p1{p1, SPRING_STIFFNESS, SPRING_INITIAL_LENGTH};
    SpringForceGenerator sfg_p2{p2, SPRING_STIFFNESS, SPRING_INITIAL_LENGTH};
    
    PointRegistry* point_registry_ = nullptr;
    ForceRegistry* force_registry_ = nullptr;
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer& renderer) override;
    void onGUI() override;
    ~Scene3() override;
};
