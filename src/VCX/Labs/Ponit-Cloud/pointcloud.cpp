#include "pointcloud.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include <cmath>
#include <glm/gtc/matrix_inverse.hpp>
#include <iostream>
#include <list>
#include <map>
#include <Open3D/Open3D.h>
#include <set>
#include <spdlog/spdlog.h>
#include <tuple>
#include <unordered_set>

void solve(VCX::Engine::SurfaceMesh & mesh, const char * p) {
    open3d::geometry::PointCloud     pc;
    open3d::io::ReadPointCloudOption po;
    open3d::io::ReadPointCloudFromPCD(p, pc, po);
    pc.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(), false);
    if (pc.HasNormals()) printf("Has normal!\n");
    printf("num points: %zu\n", pc.points_.size());
    auto d = pc.ComputeNearestNeighborDistance();
    for (int i = 0; i < d.size(); i++) { d[i] *= 1.5; }
    open3d::geometry::TriangleMesh                  m1;
    std::shared_ptr<open3d::geometry::TriangleMesh> ms;
    // ms = m1.CreateFromPointCloudBallPivoting(pc, d);
    auto out = m1.CreateFromPointCloudPoisson(pc, 8);
    ms       = std::get<0>(out);
    open3d::geometry::TriangleMesh m(*ms);
    for (auto tri : m.triangles_) {
        mesh.Indices.push_back(tri.x());
        mesh.Indices.push_back(tri.y());
        mesh.Indices.push_back(tri.z());
    }
    for (auto v : m.vertices_) { mesh.Positions.push_back(glm::vec3(1)); }
    for (int i = 0; i < m.vertices_.size(); i++) {
        mesh.Positions[i] = glm::vec3(m.vertices_[i].x(), m.vertices_[i].y(), m.vertices_[i].z());
    }
    int num_points = pc.points_.size();
}
