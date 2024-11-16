#include "Scene.h"
#include "Point.h"
#include "ForceGenerator.h"

class Scene4 : public Scene {
private:
    float step = 0.005f;
    PointRegistry* point_registry_ = nullptr;
    ForceRegistry* force_registry_ = nullptr;
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer& renderer) override;
    void onGUI() override;
};
