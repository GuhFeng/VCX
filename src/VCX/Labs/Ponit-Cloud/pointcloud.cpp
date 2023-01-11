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

void solve(VCX::Engine::SurfaceMesh & mesh, const char * p) {
    open3d::geometry::PointCloud     pc;
    open3d::io::ReadPointCloudOption po;
    open3d::io::ReadPointCloudFromPCD(p, pc, po);
    if (pc.HasNormals()) printf("Has normal!\n");
    else pc.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(), false);
    printf("num points: %zu\n", pc.points_.size());
    std::vector<double>                             radii { 0.06 };
    std::shared_ptr<open3d::geometry::TriangleMesh> m(new open3d::geometry::TriangleMesh());
    m = m->CreateFromPointCloudBallPivoting(pc, radii);
    printf("1\n");
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
    printf("vertice: %ld\ntriangles: %ld\n finish!\n", m->vertices_.size(), m->triangles_.size());
}
