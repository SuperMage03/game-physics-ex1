#include "Scene.h"
#include "Point.h"
#include "ForceGenerator.h"
#include "CollisionGenerator.h"
#include "AssignmentSpecificValues.h"

class Scene4 : public Scene {
public:
    struct SpringForcePair {
        Point& a;
        Point& b;
        SpringForceGenerator spring_force_from_a;
        SpringForceGenerator spring_force_from_b;
        SpringForcePair(Point& a, Point& b, const float& stiffness);
        SpringForcePair(Point& a, Point& b, const float& stiffness, const float& initial_length);
        void registerSpringForcePair(ForceRegistry& force_registry);
    };
private:
    float step_ = 0.005f;

    GravityForceGenerator gravity_force_generator_{glm::vec3{0.0f, 0.0f, -9.81f}};
    DampingForceGenerator damping_force_generator_{};
    BoundingBoxCollisionGenerator bounding_box_collision_generator_{BOUNDING_BOX_POSITION, BOUNDING_BOX_SCALE};
    
    PointRegistry* point_registry_ = nullptr;
    ForceRegistry* force_registry_ = nullptr;
    CollisionRegistry* collision_registry_ = nullptr;
    
    std::unique_ptr<Point> icosahedron_vertices_[ICOSAHEDRON_VERTEX_COUNT];
    std::unique_ptr<SpringForcePair> icosahedron_edges_[ICOSAHEDRON_EDGE_COUNT];

    void GenerateIcosahedron();
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer& renderer) override;
    void onGUI() override;
    ~Scene4() override;
};
