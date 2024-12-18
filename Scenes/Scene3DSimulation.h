#pragma once
#include "Scene.h"
#include <vector>

class Scene3DSimulation : public Scene {
public:
    void init() override;
    void simulateStep() override;
    void onDraw(Renderer &renderer) override;
    void onGUI() override;

private:
    int m, n, p;             
    double dx, dy, dz;       
    double nu;              
    double dt_sim;           

    bool isExplicitMethod;    

    std::vector<std::vector<std::vector<double>>> T_curr; 
    std::vector<std::vector<std::vector<double>>> T_next; 

    void initializeGrid(); 
    void simulateExplicitStep(); 
    void simulateImplicitStep(); 
};
