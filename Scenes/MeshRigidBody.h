#pragma once
#include <vector>
#include "RigidBody.h"

class MeshRigidBody : public RigidBody {
public:
    struct MeshPoint {
        float mass;
        glm::vec3 position;
    };
    void initializeData() override;
private:
    std::vector<MeshPoint> _mesh_points;
    void _calculateCenterOfMassFromMesh();
    void _calculateInertiaTensorFromMesh();
};
