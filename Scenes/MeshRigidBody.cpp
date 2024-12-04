#include "MeshRigidBody.h"
#include <cmath>

MeshRigidBody::MeshRigidBody(const glm::vec3& position, const glm::vec3& scale, const glm::quat& orientation, const std::vector<MeshPoint>& mesh_points): 
    RigidBody{position, scale, orientation}, _mesh_points{mesh_points} {
    _calculateCenterOfMassFromMesh();
    _calculateInertiaTensorFromMesh();
}

void MeshRigidBody::_calculateCenterOfMassFromMesh() {
    _mass = 0.0f;
    _position = glm::vec3(0.0f);
    for (const auto& mesh_point : _mesh_points) {
        _mass += mesh_point.mass;
        _position += mesh_point.position;
    }
    _position /= _mass;
}

void MeshRigidBody::_calculateInertiaTensorFromMesh() {
    _inertia_tensor = glm::mat3(0.0f);
    for (const auto& mesh_point : _mesh_points) {
        glm::vec3 col_0 = glm::vec3(powf(mesh_point.position.y, 2.0f) + powf(mesh_point.position.z, 2.0f), 
                                    -mesh_point.position.x * mesh_point.position.y,
                                    -mesh_point.position.x * mesh_point.position.z);
        glm::vec3 col_1 = glm::vec3(-mesh_point.position.x * mesh_point.position.y, 
                                    powf(mesh_point.position.x, 2.0f) + powf(mesh_point.position.z, 2.0f),
                                    -mesh_point.position.y * mesh_point.position.z);
        glm::vec3 col_2 = glm::vec3(-mesh_point.position.x * mesh_point.position.z, 
                                    -mesh_point.position.y * mesh_point.position.z,
                                    powf(mesh_point.position.x, 2.0f) + powf(mesh_point.position.y, 2.0f));
        mesh_point.mass * glm::mat3(col_0, col_1, col_2);
    }
}
