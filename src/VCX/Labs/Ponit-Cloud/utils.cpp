#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "pointcloud.h"
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

void LibAlg(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh, double r) {
    std::shared_ptr<open3d::geometry::TriangleMesh> m(new open3d::geometry::TriangleMesh());
    std::vector<double>                             radii { r };
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

glm::vec3       eigen2glm(Eigen::Vector3d a) { return glm::vec3(a.x(), a.y(), a.z()); }
Eigen::Vector3d glm2eigen(glm::vec3 a) { return Eigen::Vector3d(a[0], a[1], a[2]); }