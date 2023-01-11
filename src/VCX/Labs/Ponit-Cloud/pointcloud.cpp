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

void BPA(open3d::geometry::PointCloud & pc, VCX::Engine::SurfaceMesh & mesh) {
    auto   distances = pc.ComputeNearestNeighborDistance();
    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double              rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { 0.05 * rho, 0.1 * rho, 0.2 * rho, 0.5 * rho,
                                rho,        2.0 * rho, 4.0 * rho, 8 * rho };
}

void solve(
    const VCX::Engine::SurfaceMesh & old, VCX::Engine::SurfaceMesh & mesh, const char * path) {
    open3d::geometry::PointCloud pc;
    Mesh2PC(old, pc);
    LibAlg(pc, mesh, 1);
}