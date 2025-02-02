#pragma once

#include "Scene.h"
#include "RigidObjectPhysicsEngine.h"
#include "ThermodynamicPhysicsEngine.h"
#include <imgui.h>
#include <random>

class LevelThree : public Scene {
private:
    Physics::RigidObjectPhysicsEngine ROPE;
    // Size of time step
    float f_delta = 0.02;
    bool f_pause = true;
    bool gravity = true;
    Physics::IntegrationType f_integrationType = Physics::IntegrationType::MIDPOINT;

    bool f_singleStep = false;
    float f_diffusivity = 0.1;
    // Grid dimension along Ox
    int f_n = 50;
    // Grid dimension along Oy
    int f_m = 50;
    float f_X = 5.;
    float f_Y = 5.;
    bool f_changedProblem = false;
    // Wether the heatmap is currently advancing in time
    bool heatmap_change = true;
    Physics::SolverType f_solverType = Physics::SolverType::IMPLICIT;
    int f_effectRadius = 2;

    Physics::ThermodynamicPhysicsEngine TPE;
    GridFunction::ScalarGridFunction2D f_heatField;

    bool f_restart = false;

    glm::dmat4 projectionMatrix = glm::dmat4(1);
    glm::dmat4 cameraMatrix = glm::dmat4(1);
    glm::dvec3 cameraPosition = glm::dvec3(0);
    glm::dvec3 fwd = glm::dvec3(1, 0, 0);
    glm::dvec3 right = glm::dvec3(0, 1, 0);
    glm::dvec3 up = glm::dvec3(0, 0, 1);

    // Time steps completed
    int time_passed = 0;
    // Time steps you need to survive to win
    int timesteps_win = 750;
    bool won = false;
    bool lost = false;

public:
    LevelThree():
    ROPE(),
    TPE(),
    // For random colors
    gen(rd()), 
    dis(0.f, 1.f)
    {}

    void init() override {
        initialize();
    }

    // For random colors
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dis;

    void initialize() {
        ROPE = Physics::RigidObjectPhysicsEngine();
        ROPE.f_integrationType = f_integrationType;

        std::shared_ptr<RigidBall> ball1(new RigidBall(
            1.,
            0.8,
            // friction
            0.2,
            Transform3D(
                glm::dvec3(0.5, 0, 0.),
                glm::dvec3(0.4, 0.4, 0.4),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec3(0., 0., 0.),
            glm::dvec3(10., 0., 0.)
        ));

        std::shared_ptr<RigidBall> ball2(new RigidBall(
            1.,
            0.8,
            // friction
            0.2,
            Transform3D(
                glm::dvec3(-1.0, 0, 0.),
                glm::dvec3(0.4, 0.4, 0.4),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec3(0., 0., 0.),
            glm::dvec3(10., 0., 0.)
        ));

        std::shared_ptr<RigidBall> ball3(new RigidBall(
            1.,
            0.8,
            // friction
            0.2,
            Transform3D(
                glm::dvec3(-0.25, 0.75, 0.),
                glm::dvec3(0.4, 0.4, 0.4),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec3(0., 0., 0.),
            glm::dvec3(10., 0., 0.)
        ));

        // std::shared_ptr<RigidBall> ball2(new RigidBall(
        //     1.,
        //     1.0,
        //     0.0,
        //     Transform3D(
        //         glm::dvec3(0.5, 0., 0.),
        //         glm::dvec3(0.4, 0.4, 0.4),
        //         glm::dvec3(0., 0., 0.)
        //     ),
        //     glm::dvec3(0., 0., 0.),
        //     glm::dvec3(0., 0., 0.)
        // ));

        std::shared_ptr<StaticCuboid> wallB(new StaticCuboid(
            Transform3D(
                glm::dvec3(0., 0., -3.),
                glm::dvec3(f_X, f_Y, 1.),
                glm::dvec3(0., 0., 0.)
            ),
            glm::dvec4(0.8, 0.8, 0.8, 0.8)
        ));

        ball1->f_color = glm::vec4(dis(gen), dis(gen), dis(gen), 1);
        ball2->f_color = glm::vec4(dis(gen), dis(gen), dis(gen), 1);
        ball3->f_color = glm::vec4(dis(gen), dis(gen), dis(gen), 1);

        ROPE.addRigidObject(ball1);
        ROPE.addRigidObject(ball2);
        ROPE.addRigidObject(ball3);
        
        ROPE.addStaticObject(wallB);

        ROPE.setGridFunction(&f_heatField);
        
        std::cout << "+++++++++++ SCENE TEST +++++++++++" << std::endl;
        std::cout << "======== Initial system state ========" << std::endl
        << ROPE << std::endl << std::endl;

        TPE = std::move(Physics::ThermodynamicPhysicsEngine(
            // HeatProblem::HeatProblemRectDBC2D(
            //     glm::dvec2(-f_X / 2., -f_Y / 2.),
            //     glm::dvec2(f_X, f_Y),
            //     0.1,
            //     [](glm::dvec2 point, double t) {
            //         return 0.;
            //     },
            //     [](glm::dvec2 point, double t) {
            //         return 2 * sin(2. * point.x) * sin(2. * point.y) + 0.5 * ((double)(rand()) / (double)(RAND_MAX)) - 2.5;
            //     },
            //     [](double y, double t) {
            //         return 2. * (sin(y - t) + 1.) + 0.5 * (sin(5. * (y + t)) + 1.) - 2.5;
            //     },
            //     [](double y, double t) {
            //         return sin(t) * y - 2.5;
            //     },
            //     [](double x, double t) {
            //         return 2. * (sin(x - t) + 1.) + 0.5 * (sin(5. * (x + t)) + 1.) - 2.5;
            //     },
            //     [](double x, double t) {
            //         return sin(t) * x - 2.5;
            //     },
            //     HeatProblem::BoundaryConditionType::DIRICHLET,
            //     HeatProblem::BoundaryConditionType::DIRICHLET,
            //     HeatProblem::BoundaryConditionType::DIRICHLET,
            //     HeatProblem::BoundaryConditionType::DIRICHLET
            // )
            HeatProblem::HeatProblemRectDBC2D(
                glm::dvec2(-f_X / 2., -f_Y / 2.),
                glm::dvec2(f_X, f_Y),
                0.1,
                [](glm::dvec2 point, double t) {
                    return 0.;
                },
                [](glm::dvec2 point, double t) {
                    return - 4. * (point.x - 2.5) * (point.x + 2.5) * (point.y - 2.5) * (point.y + 2.5) / 39. + 2.5;
                },
                [](double y, double t) {
                    return 2.5;
                },
                [](double y, double t) {
                    return 2.5;
                },
                [](double x, double t) {
                    return 2.5;
                },
                [](double x, double t) {
                    return 2.5;
                },
                HeatProblem::BoundaryConditionType::DIRICHLET,
                HeatProblem::BoundaryConditionType::DIRICHLET,
                HeatProblem::BoundaryConditionType::DIRICHLET,
                HeatProblem::BoundaryConditionType::DIRICHLET
            )
        ));
        TPE.f_solverType = f_solverType;

        f_heatField = TPE.getInitialState(f_n, f_m);
    }

    void propagateState() {
        if(heatmap_change) {
            switch (f_solverType) {
            case Physics::SolverType::EXPLICIT:
                TPE.propagateStateExplicitOn(f_heatField, f_delta);
                break;
            case Physics::SolverType::IMPLICIT:
                TPE.propagateStateImplicitOn(f_heatField, f_delta);
                break;
            default:
                break;
            }
        }
    }

    void simulateStep() override {
        if (f_changedProblem || f_restart) {
            f_pause = true;
            initialize();
            f_changedProblem = false;
            f_restart = false;
        }
        TPE.setDiffusivity(f_diffusivity);
        if (!f_pause) {
            if (gravity) {
                ROPE.applyForceToRigidObject(0, Force(glm::dvec3(0.0, 0.0, -9.81), ROPE.getRigidObject(0)->f_transform.f_position));
                ROPE.applyForceToRigidObject(1, Force(glm::dvec3(0.0, 0.0, -9.81), ROPE.getRigidObject(1)->f_transform.f_position));
                ROPE.applyForceToRigidObject(2, Force(glm::dvec3(0.0, 0.0, -9.81), ROPE.getRigidObject(2)->f_transform.f_position));
            }
            ROPE.simulateStep(f_delta);
            propagateState();

            if(!lost) time_passed += 1;

            // If enough time has passed, player wins
            if(time_passed >= timesteps_win && lost == false) {
                won = true;
            }

            // If sphere left heatmap area, player looses
            // Add -0.4 or 0.4, because that's sphere radius, and a bit on top (tweaked the value until it seamed to fit)
            if((!ROPE.isPointInArea(0, -0.7 +  -0.5 * f_X, 0.7 + 0.5 * f_X, -0.7 + -0.5 * f_Y, 0.7 + 0.5 * f_Y)
                || !ROPE.isPointInArea(1, -0.7 +  -0.5 * f_X, 0.7 + 0.5 * f_X, -0.7 + -0.5 * f_Y, 0.7 + 0.5 * f_Y)
                || !ROPE.isPointInArea(2, -0.7 +  -0.5 * f_X, 0.7 + 0.5 * f_X, -0.7 + -0.5 * f_Y, 0.7 + 0.5 * f_Y))
                && won == false) {
                lost = true;
            }
        }
        else {
            if (f_singleStep) {
                propagateState();
                f_singleStep = false;
            }
        }
    }

    void onDraw(Renderer &renderer) override {
        projectionMatrix = renderer.camera.projectionMatrix();
        cameraMatrix = renderer.camera.viewMatrix;
        cameraPosition = renderer.camera.position;
        fwd = inverse(cameraMatrix) * glm::dvec4(0, 0, 1, 0);
        right = inverse(cameraMatrix) * glm::dvec4(1, 0, 0, 0);
        up = inverse(cameraMatrix) * glm::dvec4(0, 1, 0, 0);

        ROPE.onDraw(renderer);
        f_heatField.onDraw(renderer);
        // renderer.drawLine(glm::vec3(-0.5 * f_X,0,0), glm::vec3(0.5 * f_X,0,0), glm::vec3(0.,0.,1.));
    }

    void onGUI() override {
        ImGui::Text("------------------ GAME ------------------");
        ImGui::Text("Can you keep the balls on the heatmap long enough?");
        ImGui::Text("Time passed: %d", time_passed);
        ImGui::ProgressBar((float) time_passed / (float) timesteps_win, ImVec2(300, 0), "Victory progress :)");
        ImGui::Checkbox("Pause", &this->f_pause);
        if(won) {
            ImGui::Text("Congrats! YOU WON !!!");
        }
        else if(lost) {
            ImGui::Text("You lost. Press 'Reload scene to try again'.");
        } else {
            ImGui::Text("Game ongoing");
        }
        ImGui::Text("");
        ImGui::Text("----------------- SYSTEM -----------------");
        ImGui::SliderFloat("Delta", &this->f_delta, 0.f, 0.1);
        ImGui::Text("Space : f_pause/unpause");
        if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
            this->f_pause = !this->f_pause;
        }
        if(ImGui::IsMouseDown(ImGuiMouseButton_Right)){   
            auto drag = ImGui::GetMouseDragDelta(1);
            if(drag.x != 0 || drag.y != 0) {
                glm::dvec3 dx = (double)(drag.x) * right;
                glm::dvec3 dy = (double)(-drag.y) * up;
                ROPE.applyForceToRigidObject(
                    0,
                    Force (
                        (dx + dy) / 25.,
                        ROPE.getRigidObject(0)->f_transform.f_position
                    )
                );
            }
        }
        ImGui::SliderFloat("Diffusivity", &this->f_diffusivity, 0.001f, 1.);
        const char* schemeNames[] = {"Explicit", "Implicit"};
        ImGui::Text("N and M denote the overall amount");
        ImGui::Text("of vertices along Ox and Oy respectively,");
        ImGui::Text("including the boundary vertices");
        f_changedProblem = f_changedProblem || ImGui::SliderInt("N", &this->f_n, 3, 100);
        f_changedProblem = f_changedProblem || ImGui::SliderInt("M", &this->f_m, 3, 100);
        f_restart = ImGui::Button("Restart");
        ImGui::Text("R : Restart");
        if (ImGui::IsKeyPressed(ImGuiKey_R)) {
            f_restart = true;
        }
        ImGui::Text("S : while paused perform a single time step");
        if (ImGui::IsKeyPressed(ImGuiKey_S)) {
            this->f_singleStep = true;
        }
        ImGui::SliderInt("Manual effect radius", &this->f_effectRadius, 0, 5);
        ImGui::Text("While unpaused:");
        ImGui::Text("Hold E : Heat up depending on mouse cursor position");
        if (ImGui::IsKeyDown(ImGuiKey_E)) {
            if (!f_pause) {
                // auto mousePosition = ImGui::GetMousePos();
                // auto windowSize = ImGui::GetIO().DisplaySize;
                // double tx = (double)(mousePosition.x) / (double)(windowSize.x);
                // double ty = 1 - (double)(mousePosition.y) / (double)(windowSize.y);
                // glm::dvec2 tp(tx, ty);
                // int i = (int)(f_heatField.getN() * tp.x);
                // int j = (int)(f_heatField.getM() * tp.y);
                // for (int io = std::max(0, i - f_effectRadius); io <= i + f_effectRadius; io++) {
                //     for (int jo = std::max(0, j - f_effectRadius); jo <= j + f_effectRadius; jo++) {
                //         f_heatField.addToValue(io, jo, 0.5);
                //     }
                // }

                ImVec2 mousePos = ImGui::GetMousePos();
                ImVec2 screenSize = ImGui::GetIO().DisplaySize;

                const GridFunction::ScalarGridFunction2D::TrianglePlane* planeHit = getTrianglePlaneOnMouse(mousePos, screenSize);

                if (planeHit != nullptr) {
                    for (int io = std::max(0, planeHit->p1Index.x - f_effectRadius); io <= planeHit->p1Index.x + f_effectRadius; io++) {
                        for (int jo = std::max(0, planeHit->p1Index.y - f_effectRadius); jo <= planeHit->p1Index.y + f_effectRadius; jo++) {
                            // Third parameter: Effect of heat source
                            f_heatField.addToValue(io, jo, 0.1);
                        }
                    }
                }
            }
        }
        // ImGui::Text("Hold Q : Cool down depending on mouse cursor position");
        // if (ImGui::IsKeyDown(ImGuiKey_Q)) {
        //     if (!f_pause) {
        //         // auto mousePosition = ImGui::GetMousePos();
        //         // auto windowSize = ImGui::GetIO().DisplaySize;
        //         // double tx = (double)(mousePosition.x) / (double)(windowSize.x);
        //         // double ty = 1 - (double)(mousePosition.y) / (double)(windowSize.y);
        //         // glm::dvec2 tp(tx, ty);
        //         // int i = (int)(f_heatField.getN() * tp.x);
        //         // int j = (int)(f_heatField.getM() * tp.y);
        //         // for (int io = std::max(0, i - f_effectRadius); io <= i + f_effectRadius; io++) {
        //         //     for (int jo = std::max(0, j - f_effectRadius); jo <= j + f_effectRadius; jo++) {
        //         //         f_heatField.addToValue(io, jo, -0.5);
        //         //     }
        //         // }

        //         ImVec2 mousePos = ImGui::GetMousePos();
        //         ImVec2 screenSize = ImGui::GetIO().DisplaySize;

        //         const GridFunction::ScalarGridFunction2D::TrianglePlane* planeHit = getTrianglePlaneOnMouse(mousePos, screenSize);

        //         if (planeHit != nullptr) {
        //             for (int io = std::max(0, planeHit->p1Index.x - f_effectRadius); io <= planeHit->p1Index.x + f_effectRadius; io++) {
        //                 for (int jo = std::max(0, planeHit->p1Index.y - f_effectRadius); jo <= planeHit->p1Index.y + f_effectRadius; jo++) {
        //                     f_heatField.addToValue(io, jo, -0.1);
        //                 }
        //             }
        //         }
        //     }
        // }
    }

    std::pair<bool, double> planeRayIntersection(const glm::dvec3& planeNormal, const glm::dvec3& rayDir, const glm::dvec3& pointOnPlane, const glm::dvec3& rayOrigin) {
        double denom = glm::dot(planeNormal, rayDir);

        // If not parallel
        if (denom != 0.0) {
            double d = -glm::dot(planeNormal, pointOnPlane);
            double t = -(glm::dot(planeNormal, rayOrigin) + d) / denom;
            // Intersection is at the opposite direction of the ray
            if (t < 0) {
                return std::make_pair(false, t);
            }
            return std::make_pair(true, t); // hit, intersection value
        }
        return std::make_pair(false, 0.0);
    }

    double calculateTrianglePlaneArea(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
        return 0.5 * glm::length(glm::cross(b - a, c - a));
    }

    bool PointInSegment(const glm::dvec3& point, const glm::dvec3& a, const glm::dvec3& b) {
        glm::dvec3 direction = glm::normalize(b - a);
        double length = glm::length(b - a);
        glm::dvec3 pointRelativeToA = point - a;
        double projection = glm::dot(direction, pointRelativeToA);
        projection = std::clamp(projection, 0.0, length);
        glm::dvec3 closestPoint = a + projection * direction;
        return glm::distance2(point, closestPoint) < 0.000001;
    }

    bool PointInTriangle(const glm::dvec3& point, const GridFunction::ScalarGridFunction2D::TrianglePlane& trianglePlane) {
        // ----- Using barycentric coordinates area -----
        // double a1 = calculateTrianglePlaneArea(point, trianglePlane.p1, trianglePlane.p2);
        // double a2 = calculateTrianglePlaneArea(point, trianglePlane.p2, trianglePlane.p3);
        // double a3 = calculateTrianglePlaneArea(point, trianglePlane.p3, trianglePlane.p1);
        // double resultArea = a1 + a2 + a3;
        // double expectedArea = calculateTrianglePlaneArea(trianglePlane.p1, trianglePlane.p2, trianglePlane.p3);
        // // If resultArea and expectedArea is close enough
        // if (fabs(expectedArea - resultArea) < 0.0001) {
        //     return true;
        // }
        // return false;

        // ----- Using the normal of the three sub-triangles -----
        // Move p1, p2, p3 to be in the perspective where point is the origin
        glm::dvec3 a = trianglePlane.p1 - point;
        glm::dvec3 b = trianglePlane.p2 - point;
        glm::dvec3 c = trianglePlane.p3 - point;

        // Compute the normal vectors for triangles:
        // u = normal of PBC
        // v = normal of PCA
        // w = normal of PAB

        glm::dvec3 u = glm::cross(b, c);
        glm::dvec3 v = glm::cross(c, a);
        glm::dvec3 w = glm::cross(a, b);

        // If normal is the zero-vector, that means point is parallel with the other two points
        //   thus we need to check if point is within the line segment formed by the other two points
        if (glm::length2(u) == 0.0) {
            return PointInSegment(glm::dvec3(0.0), b, c);
        }
        if (glm::length2(v) == 0.0) {
            return PointInSegment(glm::dvec3(0.0), c, a);
        }
        if (glm::length2(w) == 0.0) {
            return PointInSegment(glm::dvec3(0.0), a, b);
        }

        // Test to see if the normals are facing the same direction, return false if not
        if (fabs(1.0 - (glm::dot(u, v) / (glm::length(u)*glm::length(v)))) > 0.001) {
            return false;
        }
        if (fabs(1.0 - (glm::dot(u, w) / (glm::length(u)*glm::length(w)))) > 0.001) {
            return false;
        }

        // All normals facing the same way, return true
        return true;
    }

    const GridFunction::ScalarGridFunction2D::TrianglePlane* getTrianglePlaneOnMouse(const ImVec2& mousePos, const ImVec2& screenSize) {
        // The ray Start and End positions, in Normalized Device Coordinates (Have you read Tutorial 4 ?)
        glm::dvec4 lRayStart_NDC(
            ((double)mousePos.x/(double)screenSize.x - 0.5) * 2.0, // [0,1024] -> [-1,1]
            -((double)mousePos.y/(double)screenSize.y - 0.5) * 2.0, // [0, 768] -> [-1,1]
            -1.0, // The near plane maps to Z=-1 in Normalized Device Coordinates
            1.0
        );
        glm::dvec4 lRayEnd_NDC(
            ((double)mousePos.x/(double)screenSize.x - 0.5) * 2.0, // [0,1024] -> [-1,1]
            -((double)mousePos.y/(double)screenSize.y - 0.5) * 2.0, // [0, 768] -> [-1,1]
            0.0, // The far plane
            1.0
        );

        glm::dvec4 lRayStart_camera = glm::inverse(projectionMatrix) * lRayStart_NDC;
        lRayStart_camera /= lRayStart_camera.w;

        glm::dvec4 lRayStart_world = glm::inverse(cameraMatrix) * lRayStart_camera;
        lRayStart_world /= lRayStart_world.w;

        glm::dvec4 lRayEnd_camera = glm::inverse(projectionMatrix) * lRayEnd_NDC;
        lRayEnd_camera /= lRayEnd_camera.w;

        glm::dvec4 lRayEnd_world = glm::inverse(cameraMatrix) * lRayEnd_camera;
        lRayEnd_world /= lRayEnd_world.w;

        glm::dvec3 lRayDir_world = glm::normalize(lRayEnd_world - lRayStart_world);

        const unsigned trianglePlanesCount = f_heatField.getTrianglePlanesCount();
        const GridFunction::ScalarGridFunction2D::TrianglePlane* trianglePlanes = f_heatField.getTrianglePlanes();


        double closestT = std::numeric_limits<double>::infinity();
        const GridFunction::ScalarGridFunction2D::TrianglePlane* planeHit = nullptr;

        for (unsigned i = 0; i < trianglePlanesCount; i++) {
            const GridFunction::ScalarGridFunction2D::TrianglePlane& curTrianglePlane = trianglePlanes[i];
            const glm::dvec3& planeNormal = curTrianglePlane.normal;
            
            auto [hasHit, t] = planeRayIntersection(planeNormal, lRayDir_world, curTrianglePlane.p1, cameraPosition);
            if (hasHit) {
                glm::dvec3 hitPoint = cameraPosition + lRayDir_world * t;
                if (PointInTriangle(hitPoint, curTrianglePlane) && (closestT > t)) {
                    closestT = t;
                    planeHit = &curTrianglePlane;
                }
            }
        }
        return planeHit;
    }
};