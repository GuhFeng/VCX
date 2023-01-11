#include "pointcloud.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include <cmath>
#include <glm/gtc/matrix_inverse.hpp>
#include <iostream>
#include <list>
#include <map>
#include <open3d/Open3D.h>
#include <set>
#include <spdlog/spdlog.h>
#include <tuple>
#include <unordered_set>

void Mesh2PC(const VCX::Engine::SurfaceMesh & mesh, open3d::geometry::PointCloud & pc) {
    auto nor = mesh.ComputeNormals();
    for (int i = 0; i < mesh.Positions.size(); i++) {
        auto p = mesh.Positions[i];
        pc.points_.push_back(Eigen::Vector3d(p[0], p[1], p[2]));
        p = nor[i];
        pc.normals_.push_back(Eigen::Vector3d(p[0], p[1], p[2]));
    }
}

void LoadData(open3d::geometry::PointCloud & pc, const std::string path) {
    open3d::io::ReadPointCloudOption po;
    open3d::io::ReadPointCloudFromPCD(path, pc, po);
}

void LibBPA(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh) {
    auto   distances = pc.ComputeNearestNeighborDistance();
    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double              rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { 0.05 * rho, 0.1 * rho, 0.2 * rho, 0.5 * rho,
                                rho,        2.0 * rho, 4.0 * rho, 8 * rho };
    std::shared_ptr<open3d::geometry::TriangleMesh> m(new open3d::geometry::TriangleMesh());
    m = m->CreateFromPointCloudBallPivoting(pc, radii);
    for (auto tri : m->triangles_) {
        mesh.Indices.push_back(tri.x());
        mesh.Indices.push_back(tri.y());
        mesh.Indices.push_back(tri.z());
    }
    for (auto v : m->vertices_) { mesh.Positions.push_back(glm::vec3(1)); }
    for (int i = 0; i < m->vertices_.size(); i++) {
        mesh.Positions[i] =
            glm::vec3(m->vertices_[i].x(), m->vertices_[i].y(), m->vertices_[i].z());
    }
}

void solve(
    const VCX::Engine::SurfaceMesh & old, VCX::Engine::SurfaceMesh & mesh, const char * path) {
    open3d::geometry::PointCloud pc;
    Mesh2PC(old, pc);
    LibBPA(pc, mesh);
}