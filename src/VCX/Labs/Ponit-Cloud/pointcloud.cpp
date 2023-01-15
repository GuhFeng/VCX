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
// #define LibImplement

void solve(const VCX::Engine::SurfaceMesh & old, VCX::Engine::SurfaceMesh & mesh, int radii) {
    open3d::geometry::PointCloud pc;
    Mesh2PC(old, pc);
    printf("Reconstruction Begin!\n");
    auto distances = pc.ComputeNearestNeighborDistance();

    double avg_distance =
        std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double rho     = 1.25 * avg_distance / 2.0;
    float  size[4] = { 0.5, 1.0, 2.0, 4.0 };
#ifndef LibImplement
    BPA_run(pc, mesh, size[radii] * rho);
#else
    LibAlg(pc, mesh, size[radii] * rho);
#endif
    printf(
        "Number of Points: %d\nNumber of Original Triangles: %d\nNumber of Reconstructed Triangles: %d\n",
        old.Positions.size(),
        old.Indices.size(),
        mesh.Indices.size());
    printf("Reconstruction Finish!\n");
}