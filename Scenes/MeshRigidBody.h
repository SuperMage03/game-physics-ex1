#pragma once
#include "RigidBody.h"
#include <vector>

class MeshRigidBody : public RigidBody {
public:
    struct MeshPoint {
        float mass;
        glm::vec3 position;
    };
    MeshRigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const std::vector<MeshPoint>& mesh_points);
private:
    std::vector<MeshPoint> _mesh_points;
    void calculateCenterOfMassFromMesh();
    void calculateInertiaTensorFromMesh();
};
