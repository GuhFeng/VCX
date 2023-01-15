#include "pointcloud.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "utils.h"
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

void solve(const VCX::Engine::SurfaceMesh & old, VCX::Engine::SurfaceMesh & mesh, int radii) {
    open3d::geometry::PointCloud pc;
    Mesh2PC(old, pc);
    BPA_run(pc, mesh, radii);
    printf(
        "Number of Points: %d\nNumber of Original Triangles: %d\nNumber of Reconstructed Triangles: %d\n",
        old.Positions.size(),
        old.Indices.size(),
        mesh.Indices.size());
}