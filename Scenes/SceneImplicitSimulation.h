#pragma once
#include "Scene.h"
#include <vector>

class SceneImplicitSimulation : public Scene {
public:
    void init() override;           
    void simulateStep() override;   
    void onDraw(Renderer &renderer) override; 
    void onGUI() override;         

private:
    int m, n;              
    double dx, dy;         
    double nu;             
    double dt_sim;         

    std::vector<std::vector<double>> T_curr; 
    std::vector<std::vector<double>> T_next; 

    std::vector<std::vector<double>> A;  
    std::vector<double> b;             

    void initializeGrid();   
    void assembleSystem();  
    void solveLinearSystem();
};
